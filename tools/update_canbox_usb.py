#!/usr/bin/env python3
"""
Automated USB Flash Drive Firmware Updater for Hiworld CAN Box.

Performs:
1. Compiles the Nuvoton NUC131 application firmware target ('raise_nuc131') using PlatformIO.
2. Converts and packages the compiled .bin into a Hiworld obfuscated IAP container ('can_app.iap').
3. Detects connected USB storage drives containing a 'canmcuupdata' directory.
4. Safely copies and flushes the IAP container to the USB drive ready for Head Unit upgrade.
"""

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

# Try importing packing logic from existing pack_hiworld_iap tool
REPO_ROOT = Path(__file__).resolve().parent.parent
TOOLS_DIR = REPO_ROOT / "tools"

if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

try:
    from pack_hiworld_iap import (
        DEFAULT_KEY,
        MAGIC_HEADER,
        PAYLOAD_SIZE,
        REC_SIZE,
        calc_record_csum,
        obfuscate_chunk,
        pack_firmware,
    )
except ImportError:
    # Standalone fallback implementation if imported independently
    REC_SIZE = 136
    PAYLOAD_SIZE = 128
    MAGIC_HEADER = bytes.fromhex("0109070800020107913247a4")
    DEFAULT_KEY = bytes([0x19, 0x84, 0x07, 0x42])

    def calc_record_csum(rec_bytes: bytes) -> int:
        return sum(
            int.from_bytes(rec_bytes[j * 4 : j * 4 + 4], "little")
            for j in range(33)
        ) & 0xFFFFFFFF

    def obfuscate_chunk(chunk: bytes, key: bytes) -> bytes:
        return bytes((chunk[i] + i + key[i & 3]) & 0xFF for i in range(len(chunk)))

    def pack_firmware(bin_path: str, out_iap_path: str, template_iap_path: str = None, key: bytes = DEFAULT_KEY):
        with open(bin_path, "rb") as f:
            payload = bytearray(f.read())

        if template_iap_path:
            with open(template_iap_path, "rb") as f:
                template = f.read()
            record_0 = bytearray(template[:REC_SIZE])
            record_0[132:136] = key
        else:
            record_0 = bytearray(REC_SIZE)
            record_0[:12] = MAGIC_HEADER
            record_0[12:128] = b"\x00" * 116
            record_0[128:132] = b"P407"
            record_0[132:136] = key

        pad_rem = len(payload) % PAYLOAD_SIZE
        if pad_rem != 0:
            payload += b"\xFF" * (PAYLOAD_SIZE - pad_rem)

        total_records = len(payload) // PAYLOAD_SIZE
        out_iap = bytearray(record_0)

        for k in range(1, total_records + 1):
            chunk = payload[(k - 1) * PAYLOAD_SIZE : k * PAYLOAD_SIZE]
            rec = bytearray(obfuscate_chunk(chunk, key))
            rec += k.to_bytes(2, "little")
            rec.append((k * 100) // total_records)
            rec.append(1 if k == total_records else 0)
            csum = calc_record_csum(rec)
            rec += csum.to_bytes(4, "little")
            out_iap += rec

        with open(out_iap_path, "wb") as f:
            f.write(out_iap)

        return len(payload), total_records, len(out_iap)


PSEUDO_FS = {
    "sysfs", "proc", "devtmpfs", "devpts", "tmpfs", "securityfs",
    "cgroup", "cgroup2", "pstore", "bpf", "autofs", "mqueue",
    "hugetlbfs", "debugfs", "tracefs", "fusectl", "configfs",
    "ramfs", "overlay", "binfmt_misc", "efivarfs", "squashfs",
}


def find_pio_binary() -> str:
    """Finds PlatformIO CLI executable."""
    candidates = [
        os.path.expanduser("~/.platformio/penv/bin/pio"),
        shutil.which("pio"),
    ]
    for c in candidates:
        if c and os.path.exists(c) and os.access(c, os.X_OK):
            return c
    return "pio"


def compile_target(pio_env: str = "raise_nuc131") -> Path:
    """Compiles the specified PlatformIO environment with -s."""
    pio_bin = find_pio_binary()
    print(f"\n[*] [1/3] Compiling firmware target '{pio_env}' (using '{pio_bin}')...")

    cmd = [pio_bin, "run", "-s", "-e", pio_env]
    start_time = time.time()
    try:
        res = subprocess.run(cmd, cwd=str(REPO_ROOT), check=False)
    except FileNotFoundError:
        print(f"[-] PlatformIO executable '{pio_bin}' not found in PATH or ~/.platformio.")
        print("    Install PlatformIO or ensure it is accessible.")
        sys.exit(1)

    elapsed = time.time() - start_time
    if res.returncode != 0:
        print(f"[-] PlatformIO build failed with return code {res.returncode}.")
        sys.exit(res.returncode)

    bin_path = REPO_ROOT / ".pio" / "build" / pio_env / "program.bin"
    if not bin_path.exists():
        print(f"[-] Expected binary not found at '{bin_path}'!")
        sys.exit(1)

    bin_size = bin_path.stat().st_size
    print(f"[+] Build succeeded in {elapsed:.2f}s: '{bin_path}' ({bin_size:,} bytes)")
    return bin_path


def convert_to_iap(bin_path: Path, out_iap: Path, template_iap: str = None) -> Path:
    """Packages the raw binary into Hiworld can_app.iap container."""
    print(f"\n[*] [2/3] Converting binary to Hiworld IAP container...")
    out_iap.parent.mkdir(parents=True, exist_ok=True)

    pack_firmware(str(bin_path), str(out_iap), template_iap_path=template_iap)
    iap_size = out_iap.stat().st_size
    records = iap_size // REC_SIZE
    print(f"[+] Successfully generated '{out_iap}' ({iap_size:,} bytes, {records} records)")
    return out_iap


def decode_octal_escapes(s: str) -> str:
    """Decodes octal escapes in /proc/mounts (e.g. \\040 -> space)."""
    return re.sub(r"\\([0-7]{3})", lambda m: chr(int(m.group(1), 8)), s)


def find_usb_candidates() -> list[tuple[Path, list[Path]]]:
    """
    Finds mounted external/removable storage roots and any 'canmcuupdata' directories inside.
    Returns list of tuples: (mount_root, [list of matching canmcuupdata paths])
    """
    mount_points: list[Path] = []

    # 1. Inspect Linux /proc/mounts
    if os.path.exists("/proc/mounts"):
        try:
            with open("/proc/mounts", "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        _dev, mnt, fstype = parts[0], parts[1], parts[2]
                        if fstype in PSEUDO_FS:
                            continue
                        decoded = decode_octal_escapes(mnt)
                        mount_points.append(Path(decoded))
        except Exception:
            pass

    # 2. Inspect typical external mount root directories
    user = os.environ.get("USER") or os.environ.get("LOGNAME")
    check_roots = ["/run/media", "/media", "/mnt"]
    if user:
        check_roots.extend([f"/run/media/{user}", f"/media/{user}"])

    for r in check_roots:
        rp = Path(r)
        if rp.is_dir():
            try:
                for entry in rp.iterdir():
                    if entry.is_dir():
                        mount_points.append(entry)
            except (PermissionError, OSError):
                pass

    # 3. macOS external volumes
    macos_vols = Path("/Volumes")
    if macos_vols.is_dir():
        try:
            for entry in macos_vols.iterdir():
                if entry.is_dir():
                    mount_points.append(entry)
        except (PermissionError, OSError):
            pass

    # 4. Windows drive letters
    if sys.platform.startswith("win"):
        for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
            drive = Path(f"{letter}:\\")
            if drive.exists():
                mount_points.append(drive)

    # Filter, resolve, and scan for canmcuupdata
    results: list[tuple[Path, list[Path]]] = []
    seen = set()

    for m in mount_points:
        try:
            resolved = m.resolve()
            if resolved in seen or not resolved.is_dir():
                continue
            seen.add(resolved)

            matches = []
            try:
                for child in resolved.iterdir():
                    if child.is_dir() and child.name.lower() == "canmcuupdata":
                        matches.append(child)
            except (PermissionError, OSError):
                continue

            results.append((resolved, matches))
        except (PermissionError, OSError):
            continue

    return results


def calculate_sha256(path: Path) -> str:
    """Calculates SHA-256 hex digest of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(65536):
            h.update(chunk)
    return h.hexdigest()


def copy_iap_safely(src_iap: Path, target_dir: Path, target_filename: str = "can_app.iap") -> Path:
    """Copies IAP container to target directory with safe sync."""
    target_dir.mkdir(parents=True, exist_ok=True)
    dst_file = target_dir / target_filename

    print(f"[*] Copying '{src_iap.name}' -> '{dst_file}'...")
    with open(src_iap, "rb") as fsrc, open(dst_file, "wb") as fdst:
        shutil.copyfileobj(fsrc, fdst, length=65536)
        fdst.flush()
        os.fsync(fdst.fileno())

    # Flush kernel filesystem dirty pages to USB
    if hasattr(os, "sync"):
        os.sync()

    # Verify write integrity
    src_hash = calculate_sha256(src_iap)
    dst_hash = calculate_sha256(dst_file)

    if src_hash != dst_hash:
        raise IOError(f"Checksum mismatch after copying to '{dst_file}'! USB write may have failed.")

    return dst_file


def sync_drive() -> None:
    """Flushes kernel filesystem caches and syncs storage devices."""
    print("[*] Synchronizing filesystem caches to disk (sync)...")
    if hasattr(os, "sync"):
        os.sync()
    sync_bin = shutil.which("sync")
    if sync_bin:
        try:
            subprocess.run([sync_bin], check=False)
        except Exception:
            pass


def get_mount_info(path: Path) -> tuple[Path | None, str | None]:
    """
    Finds the mount point and block device for a given filesystem path.
    Returns (mount_point, source_device).
    """
    target = path.resolve()

    # 1. Try /proc/mounts on Linux
    if os.path.exists("/proc/mounts"):
        best_mnt: Path | None = None
        best_dev: str | None = None
        best_len = -1
        try:
            with open("/proc/mounts", "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        dev, mnt = parts[0], parts[1]
                        decoded_mnt = decode_octal_escapes(mnt)
                        mnt_path = Path(decoded_mnt).resolve()
                        if target == mnt_path or mnt_path in target.parents:
                            if len(str(mnt_path)) > best_len:
                                best_len = len(str(mnt_path))
                                best_mnt = mnt_path
                                best_dev = dev
            if best_mnt:
                return best_mnt, best_dev
        except Exception:
            pass

    # 2. Try findmnt if available
    findmnt = shutil.which("findmnt")
    if findmnt:
        try:
            res = subprocess.run(
                [findmnt, "-n", "-o", "TARGET,SOURCE", "--target", str(target)],
                capture_output=True,
                text=True,
                check=False,
            )
            if res.returncode == 0 and res.stdout.strip():
                lines = res.stdout.strip().splitlines()
                parts = lines[0].split()
                if len(parts) >= 2:
                    return Path(parts[0]).resolve(), parts[1]
                elif len(parts) == 1:
                    return Path(parts[0]).resolve(), None
        except Exception:
            pass

    # 3. Fallback: walk up parents checking os.path.ismount
    curr = target
    while not os.path.ismount(str(curr)) and curr.parent != curr:
        curr = curr.parent
    if os.path.ismount(str(curr)):
        return curr, None

    return None, None


def is_protected_mount(mount_point: Path) -> bool:
    """Checks if a mount point is a critical system path or contains the repository/working dir."""
    resolved = mount_point.resolve()
    protected_exact = {
        Path("/"),
        Path("/home"),
        Path("/root"),
        Path("/boot"),
        Path("/boot/efi"),
        Path("/etc"),
        Path("/usr"),
        Path("/var"),
        Path("/tmp"),
        Path("/opt"),
        Path("/run"),
        Path("/media"),
        Path("/mnt"),
    }
    if resolved in protected_exact:
        return True

    repo_root = REPO_ROOT.resolve()
    if resolved == repo_root or resolved in repo_root.parents:
        return True

    cwd = Path.cwd().resolve()
    if resolved == cwd or resolved in cwd.parents:
        return True

    return False


def unmount_drive(mount_point: Path, dev: str | None = None) -> bool:
    """Attempts to cleanly unmount the given storage device/mount point."""
    if is_protected_mount(mount_point):
        print(f"[*] Skipping unmount of system/working mount point: '{mount_point}'")
        return False

    target_desc = f"'{mount_point}'" + (f" ({dev})" if dev else "")
    print(f"[*] Unmounting USB drive at {target_desc}...")

    # 1. Linux udisksctl (preferred for user-mounted removable devices, does not require root)
    if dev and dev.startswith("/dev/") and shutil.which("udisksctl"):
        try:
            res = subprocess.run(
                ["udisksctl", "unmount", "-b", dev],
                capture_output=True,
                text=True,
                check=False,
            )
            if res.returncode == 0:
                print(f"[✓] Successfully unmounted '{dev}' via udisksctl.")
                return True
        except Exception:
            pass

    # 2. Standard umount command
    umount_bin = shutil.which("umount")
    if umount_bin:
        try:
            res = subprocess.run(
                [umount_bin, str(mount_point)],
                capture_output=True,
                text=True,
                check=False,
            )
            if res.returncode == 0:
                print(f"[✓] Successfully unmounted '{mount_point}' via umount.")
                return True
            else:
                err = res.stderr.strip()
                if err:
                    print(f"[-] umount error: {err}")
        except Exception as e:
            print(f"[-] umount failed: {e}")

    # 3. macOS diskutil
    if sys.platform == "darwin" and shutil.which("diskutil"):
        try:
            res = subprocess.run(
                ["diskutil", "unmount", str(mount_point)],
                capture_output=True,
                text=True,
                check=False,
            )
            if res.returncode == 0:
                print(f"[✓] Successfully unmounted '{mount_point}' via diskutil.")
                return True
        except Exception:
            pass

    print(f"[-] Warning: Could not automatically unmount '{mount_point}'. Please eject manually.")
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Compile NUC131 firmware, convert to Hiworld IAP container, and write to USB flash drive."
    )
    parser.add_argument(
        "-e", "--env",
        default="raise_nuc131",
        help="PlatformIO environment to compile (default: 'raise_nuc131')"
    )
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="Skip compilation and use existing .bin"
    )
    parser.add_argument(
        "-b", "--bin",
        default=None,
        help="Custom .bin file path (overrides default PlatformIO output path)"
    )
    parser.add_argument(
        "-t", "--target",
        default=None,
        help="Manually specify target USB drive mount point, 'canmcuupdata' folder, or full .iap destination path"
    )
    parser.add_argument(
        "--create-dir",
        action="store_true",
        help="If a USB drive is found but lacks 'canmcuupdata', create it automatically"
    )
    parser.add_argument(
        "--template",
        default=None,
        help="Optional OEM Hiworld .iap template to inherit container header metadata"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run compilation and IAP generation without writing to USB drive"
    )
    parser.add_argument(
        "--no-unmount",
        action="store_true",
        help="Skip automatic unmount of USB drive after copying"
    )

    args = parser.parse_args()

    # Step 1: Compile firmware
    if args.no_build:
        if args.bin:
            bin_path = Path(args.bin).resolve()
        else:
            bin_path = REPO_ROOT / ".pio" / "build" / args.env / "program.bin"
        if not bin_path.exists():
            print(f"[-] Binary '{bin_path}' does not exist! Please compile first or remove --no-build.")
            sys.exit(1)
        print(f"[*] [1/3] Skipping build step (using existing '{bin_path}', {bin_path.stat().st_size:,} bytes).")
    else:
        bin_path = compile_target(args.env)

    # Step 2: Convert to IAP container
    local_iap_path = REPO_ROOT / ".pio" / "build" / args.env / "can_app.iap"
    convert_to_iap(bin_path, local_iap_path, template_iap=args.template)

    if args.dry_run:
        print("\n[+] [DRY RUN] Finished build and IAP conversion. Skipping USB deployment.")
        return

    # Step 3: Locate target USB drive / canmcuupdata
    print(f"\n[*] [3/3] Locating USB flash drive with 'canmcuupdata' directory...")

    targets_to_write: list[Path] = []

    if args.target:
        user_target = Path(args.target).resolve()
        if user_target.is_file() or user_target.suffix.lower() == ".iap":
            targets_to_write.append(user_target.parent)
        elif user_target.name.lower() == "canmcuupdata":
            targets_to_write.append(user_target)
        else:
            # Check if user passed the mount root or general directory
            sub = user_target / "canmcuupdata"
            if sub.is_dir() or args.create_dir:
                targets_to_write.append(sub)
            else:
                targets_to_write.append(user_target)
    else:
        mounts_and_matches = find_usb_candidates()

        for mount_root, matches in mounts_and_matches:
            if matches:
                targets_to_write.extend(matches)

        if not targets_to_write and args.create_dir:
            # Check for candidate removable mounts to create directory
            for mount_root, _ in mounts_and_matches:
                # Prefer /run/media, /media, /Volumes, or removable drives
                s = str(mount_root)
                if any(k in s for k in ["/run/media", "/media", "/Volumes"]) or sys.platform.startswith("win"):
                    targets_to_write.append(mount_root / "canmcuupdata")
                    break

    if not targets_to_write:
        print("[-] Error: No connected USB drive with a 'canmcuupdata' directory was found!")
        print("\n    To resolve:")
        print("    1. Plug in your FAT32 USB flash drive.")
        print("    2. Ensure a directory named 'canmcuupdata' exists on the USB drive root.")
        print("       (e.g., /media/user/DRIVE_NAME/canmcuupdata or /run/media/user/DRIVE_NAME/canmcuupdata)")
        print("    3. Or rerun this command with '--create-dir' or '--target <path>'.")

        # Display discovered mounts as hints
        if 'mounts_and_matches' in locals():
            removable_hints = [
                str(m) for m, _ in mounts_and_matches
                if any(k in str(m) for k in ["/run/media", "/media", "/Volumes", "/mnt"])
            ]
            if removable_hints:
                print("\n    Discovered external mounts without 'canmcuupdata':")
                for h in removable_hints:
                    print(f"      - {h}")

        sys.exit(1)

    # Perform deployment to all matched targets
    successful_copies = []
    for target_dir in targets_to_write:
        try:
            copied_file = copy_iap_safely(local_iap_path, target_dir, target_filename="can_app.iap")
            successful_copies.append({
                "path": copied_file,
                "size": copied_file.stat().st_size,
                "sha256": calculate_sha256(copied_file),
            })
        except Exception as e:
            print(f"[-] Failed copying to '{target_dir}': {e}")

    if not successful_copies:
        print("[-] Deployment failed for all targets.")
        sys.exit(1)

    print("\n" + "=" * 68)
    print(" >>> CANBOX FIRMWARE UPDATE PREPARED SUCCESSFULLY <<< ")
    print("=" * 68)
    for c in successful_copies:
        print(f" [✓] Destination : {c['path']}")
        print(f"     Size        : {c['size']:,} bytes")
        print(f"     SHA-256     : {c['sha256'][:16]}...")
    print("-" * 68)

    # Step 4: Flush cache to storage and cleanly unmount
    sync_drive()

    unmounted_mounts = []
    if not args.no_unmount:
        seen_mounts = set()
        for c in successful_copies:
            mount_point, dev = get_mount_info(c["path"])
            if mount_point and mount_point not in seen_mounts:
                seen_mounts.add(mount_point)
                if unmount_drive(mount_point, dev):
                    unmounted_mounts.append(mount_point)

    print("-" * 68)
    print(" Next Steps on Android Head Unit:")
    if unmounted_mounts:
        print(" 1. USB drive is safely synced and unmounted. Unplug it and insert into the Android Head Unit.")
    else:
        print(" 1. Safely unmount/eject the USB drive and plug it into the Android Head Unit.")
    print(" 2. Open: Factory / Car Settings -> CAN Settings / Protocol Update.")
    print(" 3. Tap 'CAN MCU Update' (the button should become active).")
    print(" 4. Wait for the update progress to reach 100% and the box to reboot.")
    print("=" * 68)


if __name__ == "__main__":
    main()

