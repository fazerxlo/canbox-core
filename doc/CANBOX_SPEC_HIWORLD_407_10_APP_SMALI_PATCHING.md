# QF_Canbus.apk Smali Patching Guide for PSA TPMS

This document outlines the procedure to modify the factory `QF_Canbus.apk` to support Hiworld Universal Numeric TPMS (`0x66`) on the Peugeot/Citroen (`wc`) protocol parser.

## 1. Prerequisites (Laptop Setup)

You will need the following tools installed on your development machine:
*   **Java (JDK):** Required to run Apktool. (e.g., `sudo apt install default-jdk`)
*   **ADB (Android Debug Bridge):** For communicating with the Head Unit. (e.g., `sudo apt install adb`)
*   **Apktool:** For decompiling and recompiling the APK.
    *   Download from [ibotpeaches.github.io/Apktool/](https://ibotpeaches.github.io/Apktool/)
    *   Place `apktool.jar` and the `apktool` wrapper script in `/usr/local/bin/` and make them executable.
*   **Text Editor:** VS Code or any standard text editor.

## 2. Connecting to the Head Unit via ADB

Most Android Head Units do not have a dedicated USB-C debug port, so the easiest method is **Wireless ADB (TCP/IP)** over Wi-Fi.

1.  Connect your laptop and the Head Unit to the **same Wi-Fi network** (or hotspot).
2.  On the Head Unit, go to **Settings > About Device**.
3.  Tap **Build Number** 7 times to enable Developer Options.
4.  Go back, open **Developer Options**, and enable **USB Debugging** (and Wireless Debugging if available).
5.  Find the Head Unit's IP address (Settings > Network > Wi-Fi).
6.  On your laptop, run:
    ```bash
    adb connect <HEAD_UNIT_IP>:5555
    ```
7.  A prompt will appear on the Head Unit screen asking to allow USB debugging from your computer's RSA key. Check "Always allow" and tap **OK**.
8.  Verify the connection:
    ```bash
    adb devices
    ```
    *(You should see your device listed as `device`)*

## 3. Extracting the Original APK

You need to locate the `QF_Canbus.apk` on the system partition and pull it.

1.  Find the exact path of the APK:
    ```bash
    adb shell pm path com.qf.vehicle
    ```
    *(Output will look something like: `package:/system/app/QF_Canbus/QF_Canbus.apk` or `/oem/app/...`)*
2.  Pull the file to your laptop:
    ```bash
    adb pull /system/app/QF_Canbus/QF_Canbus.apk .
    ```
3.  **⚠️ CRITICAL STEP:** Make a backup immediately.
    ```bash
    cp QF_Canbus.apk QF_Canbus.apk.backup
    ```

## 4. Decompiling and Patching

1.  **Decompile the APK:**
    ```bash
    apktool d QF_Canbus.apk
    ```
    This creates a folder named `QF_Canbus`.

2.  **Locate the Parser:**
    Open the file `QF_Canbus/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali` in your text editor.

3.  **Add the `switch` Case:**
    Search for the `parseData` method. Scroll down slightly to find the `sparse-switch` or `packed-switch` block. Add `0x66` (which is `102` in decimal, or `0x66` hex depending on how it's formatted in your specific smali file):
    ```smali
        .sparse-switch
            ...
            0x66 -> :sswitch_tpms
            ...
        .end sparse-switch
    ```
    Then, just below the switch block where all the `:sswitch_XX` targets are defined, inject the routing call:
    ```smali
        :sswitch_tpms
        invoke-static {p0, p1, p2}, Lcom/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser;->parseTyreState(Lcom/qf/vehicle/defines/AbsVehicleDataRule;[BLcom/qf/vehicle/view/callbacks/OnDataChangeCallBack;)V
        goto :goto_0
    ```
    *(Note: Pay attention to the `goto` label. Make sure it matches the return label used by the other switch cases, usually `:goto_0` or `:cond_0`)*

4.  **Inject the `parseTyreState` Method:**
    Scroll to the very bottom of the file and paste this block exactly as is:
    ```smali
    .method private static final parseTyreState(Lcom/qf/vehicle/defines/AbsVehicleDataRule;[BLcom/qf/vehicle/view/callbacks/OnDataChangeCallBack;)V
        .locals 5
        .param p0, "absVehicleDataRule"    # Lcom/qf/vehicle/defines/AbsVehicleDataRule;
        .param p1, "bArr"    # [B
        .param p2, "onDataChangeCallBack"    # Lcom/qf/vehicle/view/callbacks/OnDataChangeCallBack;

        invoke-virtual {p0}, Lcom/qf/vehicle/defines/AbsVehicleDataRule;->getDataStartIndex()I
        move-result v0

        new-instance v1, Lcom/qf/vehicle/entity/TyreState;
        invoke-direct {v1}, Lcom/qf/vehicle/entity/TyreState;-><init>()V

        # FL Pressure
        add-int/lit8 v2, v0, 0x1
        aget-byte v2, p1, v2
        and-int/lit16 v2, v2, 0xff
        int-to-float v2, v2
        const/high16 v3, 0x41200000    # 10.0f
        div-float v2, v2, v3
        iput v2, v1, Lcom/qf/vehicle/entity/TyreState;->mPressure_LeftFront:F

        # FR Pressure
        add-int/lit8 v2, v0, 0x2
        aget-byte v2, p1, v2
        and-int/lit16 v2, v2, 0xff
        int-to-float v2, v2
        div-float v2, v2, v3
        iput v2, v1, Lcom/qf/vehicle/entity/TyreState;->mPressure_RightFront:F

        # RL Pressure
        add-int/lit8 v2, v0, 0x3
        aget-byte v2, p1, v2
        and-int/lit16 v2, v2, 0xff
        int-to-float v2, v2
        div-float v2, v2, v3
        iput v2, v1, Lcom/qf/vehicle/entity/TyreState;->mPressure_LeftBack:F

        # RR Pressure
        add-int/lit8 v2, v0, 0x4
        aget-byte v2, p1, v2
        and-int/lit16 v2, v2, 0xff
        int-to-float v2, v2
        div-float v2, v2, v3
        iput v2, v1, Lcom/qf/vehicle/entity/TyreState;->mPressure_RightBack:F

        invoke-virtual {p2, v1}, Lcom/qf/vehicle/view/callbacks/OnDataChangeCallBack;->notifyTyreBean(Lcom/qf/vehicle/entity/TyreState;)V

        return-void
    .end method
    ```

5.  **Rebuild the APK:**
    ```bash
    apktool b QF_Canbus -o QF_Canbus_Patched.apk
    ```

## 5. Pushing the Patched APK to the Head Unit

Because `QF_Canbus.apk` is a system app, you cannot install it with `adb install`. You must replace the file manually. You also **do not need to sign it**, as replacing an existing system app with an unsigned apktool build retains the original signature in `META-INF` which the system accepts.

1.  Gain root access and remount the system as Read/Write:
    ```bash
    adb root
    adb remount
    ```
    *(If `adb root` fails, your Head Unit is not rooted, and you cannot do this without rooting it first. Most Chinese head units are rooted out-of-the-box.)*
2.  Push the patched APK to the exact path you discovered in Step 3:
    ```bash
    adb push QF_Canbus_Patched.apk /system/app/QF_Canbus/QF_Canbus.apk
    ```
3.  Fix the permissions so Android can execute it:
    ```bash
    adb shell chmod 644 /system/app/QF_Canbus/QF_Canbus.apk
    ```
4.  Reboot the Head Unit:
    ```bash
    adb reboot
    ```

## 6. Verification

1.  Once the Head Unit reboots, open the built-in Vehicle/Car Info app and navigate to the TPMS screen.
2.  Run your Python simulation script on your laptop:
    ```bash
    python tools/hiworld_tpms_test.py
    ```
3.  As soon as the script injects the `0x361` frames (which your C firmware translates to `0x66` Hiworld frames over UART), the TPMS UI panel should instantly update from `--.-` to `2.4 Bar` (or whatever values your script sends).

## ⚠️ Important Warnings

*   **Bootloops & Black Screens:** If you make a syntax error in the Smali code, the `QF_Canbus` app will crash repeatedly on boot. Since it handles reverse cameras and AC overlays, a crashing Canbus app can make the Head Unit feel unresponsive or stuck on a black screen.
*   **Recovery:** If the app crashes, **do not panic and do not unplug the Head Unit.** Your Wi-Fi ADB connection will usually still start up in the background. Simply reconnect `adb connect <IP>` and push your backup file:
    ```bash
    adb root
    adb remount
    adb push QF_Canbus.apk.backup /system/app/QF_Canbus/QF_Canbus.apk
    adb shell chmod 644 /system/app/QF_Canbus/QF_Canbus.apk
    adb reboot
    ```
