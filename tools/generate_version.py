import os
import datetime

Import("env")

# Check if explicit version was provided via environment variable
version = os.environ.get("CANBOX_BUILD_VERSION") or os.environ.get("CANBOX_VERSION")
if not version:
    now = datetime.datetime.now()
    version = now.strftime("CANBOX-CORE-V%Y%m%d.%H%M%S")

import sys
sys.stdout.write(f"\n>>> [canbox-core] Firmware version embedded: {version} <<<\n\n")
sys.stdout.flush()

env.Append(CPPDEFINES=[
    ("CANBOX_BUILD_VERSION", f'\\"{version}\\"')
])
