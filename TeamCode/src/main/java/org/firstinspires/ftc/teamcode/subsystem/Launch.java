package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launch extends SubsystemBase {
    public Servo launch;
    private final Telemetry telemetry;
    public static double lockPos = 0.32;
    public static double launchPos = 0.4;

    public Launch(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        launch = hardwareMap.get(Servo.class, "drone");//0.1->0.2 claw up
        lockPos();
    }

    @Override
    public void periodic() {
        telemetry.addData("LaunchServo", launch.getPosition());
    }

    public void lockPos() {
        launch.setPosition(lockPos);
    }
    public void launchPos() {
        launch.setPosition(launchPos);
    }
}
