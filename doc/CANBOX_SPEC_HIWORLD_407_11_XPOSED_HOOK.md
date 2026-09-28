# Option A: The Xposed TPMS Hook

Because the Head Unit's `QF_Canbus.apk` relies on the system signature (`android.uid.system`), Smali patching the APK directly usually causes Android to reject it on boot. 

The cleanest software solution is an **Xposed Module**. This allows you to hook into the Head Unit's memory and inject the `0x66` (TPMS) parser on the fly, leaving the original `QF_Canbus.apk` file completely untouched and mathematically identical to the system's expected signature.

## 1. Prerequisites
1. **Magisk & LSPosed:** Your Head Unit must be rooted with Magisk, and the LSPosed framework must be installed and active.
2. **Android Studio:** To compile the Xposed Module.
3. **Xposed API:** Add `compileOnly 'de.robv.android.xposed:api:82'` to your `build.gradle`.

## 2. The Hook Strategy
We will intercept the `parseData` method inside `com.qf.vehicle.band.peugeot.parse.wc.PeugeotDataParser`. 
When the CAN box sends the `0x66` TPMS packet, our hook will catch it *before* the original method drops it, parse the bytes into a `TyreState` object, and push it to the UI using reflection.

## 3. The Xposed Java Implementation

Create a new Android project with no Activity, and use this exact class:

```java
package com.fazer.canbox.tpmshook;

import de.robv.android.xposed.IXposedHookLoadPackage;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import de.robv.android.xposed.callbacks.XC_LoadPackage.LoadPackageParam;

public class TpmsHook implements IXposedHookLoadPackage {
    
    @Override
    public void handleLoadPackage(final LoadPackageParam lpparam) throws Throwable {
        // Only inject into the CAN bus app
        if (!lpparam.packageName.equals("com.qf.vehicle")) {
            return;
        }

        XposedBridge.log("QF_Canbus Peugeot TPMS Hook Loaded!");

        // Find the Hiworld Peugeot parser class
        Class<?> peugeotParserClass = XposedHelpers.findClass(
            "com.qf.vehicle.band.peugeot.parse.wc.PeugeotDataParser", lpparam.classLoader);

        // Hook the main parseData router
        XposedHelpers.findAndHookMethod(peugeotParserClass, "parseData",
                "com.qf.vehicle.defines.AbsVehicleDataRule", 
                byte[].class, 
                "com.qf.vehicle.view.callbacks.OnDataChangeCallBack", 
                String.class,
                new XC_MethodHook() {
                    @Override
                    protected void beforeHookedMethod(MethodHookParam param) throws Throwable {
                        Object rule = param.args[0];
                        byte[] data = (byte[]) param.args[1];
                        Object callback = param.args[2];

                        // Extract the command ID using the rule engine
                        int dataType = (Integer) XposedHelpers.callMethod(rule, "getDataType", data);
                        
                        // 102 (decimal) == 0x66 (Hiworld TPMS Command)
                        if (dataType == 102 || dataType == 0x66) { 
                            int startIndex = (Integer) XposedHelpers.callMethod(rule, "getDataStartIndex");

                            // Reflectively create the TyreState object
                            Class<?> tyreStateClass = XposedHelpers.findClass(
                                "com.qf.vehicle.entity.TyreState", lpparam.classLoader);
                            Object tyreState = XposedHelpers.newInstance(tyreStateClass);

                            // Parse FL Pressure (payload byte 1)
                            float fl = ((float) (data[startIndex + 1] & 0xFF)) / 10.0f;
                            XposedHelpers.setFloatField(tyreState, "mPressure_LeftFront", fl);

                            // Parse FR Pressure (payload byte 2)
                            float fr = ((float) (data[startIndex + 2] & 0xFF)) / 10.0f;
                            XposedHelpers.setFloatField(tyreState, "mPressure_RightFront", fr);

                            // Parse RL Pressure (payload byte 3)
                            float rl = ((float) (data[startIndex + 3] & 0xFF)) / 10.0f;
                            XposedHelpers.setFloatField(tyreState, "mPressure_LeftBack", rl);

                            // Parse RR Pressure (payload byte 4)
                            float rr = ((float) (data[startIndex + 4] & 0xFF)) / 10.0f;
                            XposedHelpers.setFloatField(tyreState, "mPressure_RightBack", rr);

                            // Send the parsed object directly to the UI callback
                            XposedHelpers.callMethod(callback, "notifyTyreBean", tyreState);

                            // Terminate the method early so the original dropped-packet logic doesn't run
                            param.setResult(null);
                        }
                    }
                });
    }
}
```

## 4. `AndroidManifest.xml` Configuration
You must declare the app as an Xposed module in your manifest inside the `<application>` block:

```xml
<meta-data
    android:name="xposedmodule"
    android:value="true" />
<meta-data
    android:name="xposeddescription"
    android:value="Injects missing TPMS logic into Peugeot Hiworld CAN parser" />
<meta-data
    android:name="xposedminversion"
    android:value="93" />
```

## 5. Deployment
1. Build the APK in Android Studio.
2. Install the APK on the Head Unit.
3. Open the **LSPosed Manager** app.
4. Go to Modules, enable your `TpmsHook` module, and make sure `com.qf.vehicle` is checked in the scope.
5. **Reboot the Head Unit.**
6. Send the `0x66` frames from your C99 firmware, and the TPMS UI will instantly populate!
