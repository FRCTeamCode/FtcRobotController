package org.firstinspires.ftc.teamcode.hardware;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.AutoConstants;

import java.text.DecimalFormat;

public class NavxMicro {
    private static AHRS navx2micro;
    DecimalFormat df = new DecimalFormat("#.##");
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private boolean calibration_complete = false;
    public NavxMicro (HardwareMap hardwareMap, Telemetry telemetry) {

        navx2micro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx2micro.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        telemetry.addData("navX-Micro", "Calibration finished");
        boolean connected = navx2micro.isConnected();
        telemetry.addData("1 navX-Device", connected ? "Connected" : "Disconnected" );
//        navx2micro.zeroYaw();
        AutoConstants.isInitNavxMicro2 = true;
    }

    public static double getYaw() {
        return navx2micro.getYaw();
    }
    public static boolean isConnected() {
        return navx2micro.isConnected();
    }

    public  static void resetYaw() {
        navx2micro.zeroYaw();
    }
}


// navXTimer.reset();
//         navXTimer.start();
//         System.out.println("NavX is calibrating");
//         while (navX.isCalibrating() && (navXTimer.get() <= 7.0))
//         {
//         }
//
//         if (!navX.isCalibrating())
//         {
//         System.out.println("NavX is done calibrating" + navXTimer.get());
//         System.out.println("Firmware version: " + navX.getFirmwareVersion());
//         navXIsCalibrated = true;
//         navX.reset();
//         }
//         else
//         {
//         System.out.println("NavX calibration timed out");
//         }