package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends SubsystemBase {
    public Servo clawServo;
    private final Telemetry telemetry;
    public static double lowerClaw = 0.15;
    public static double lowClaw = 0.35;//
    public static double lowMidClaw = 0.27;
    public static double middleLowClaw = 0.375;
    public static double middleClaw = 0.735;//
    public static double highClaw = 0.84;
    public static double highLowerClaw = 0.394;
    public static double pixelPutLowClaw = 0.735;
    public static double pixelPutMidClaw = 0.775;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawServo = hardwareMap.get(Servo.class, "goB1");//0.1->0.2 claw up
        middleClaw();
    }

    @Override
    public void periodic() {
        telemetry.addData("ClawServo", clawServo.getPosition());
    }

    public void lowerClaw() {
        clawServo.setPosition(lowerClaw);
    }
    public void lowClaw() {
        clawServo.setPosition(lowClaw);
    }public void lowMidClaw() {
        clawServo.setPosition(lowMidClaw);
    }
    public void middleClaw() {
        clawServo.setPosition(middleClaw);
    }
    public void highClaw() {
        clawServo.setPosition(highClaw);
    }
    public void highLowerClaw() {
        clawServo.setPosition(highLowerClaw);
    }
    public void middleLowClaw() {
        clawServo.setPosition(middleLowClaw);
    }
    public void pixelPutLowClaw() {
        clawServo.setPosition(pixelPutLowClaw);
    }
    public void pixelPutMidClaw() {
        clawServo.setPosition(pixelPutMidClaw);
    }
}
