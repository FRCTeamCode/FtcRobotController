package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends SubsystemBase {
    public Servo clawServo;
    private final Telemetry telemetry;
    public static double lowClaw = 0.314;
    public static double middleClaw = 0.383;
    public static double highClaw = 0.4;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawServo = hardwareMap.get(Servo.class, "goB1");//0.1->0.2 claw up
        middleClaw();
    }

    @Override
    public void periodic() {
        telemetry.addData("ClawServo", clawServo.getPosition());
    }

    public void lowClaw() {
        clawServo.setPosition(lowClaw);
    }
    public void middleClaw() {
        clawServo.setPosition(middleClaw);
    }
    public void highClaw() {
        clawServo.setPosition(highClaw);
    }
}
