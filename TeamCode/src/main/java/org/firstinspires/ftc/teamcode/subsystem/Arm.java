package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    public Servo rotateServo;
    public Servo clawServo;
    private final Telemetry telemetry;
    //CONSTANTS
    public static double initPos = 0.1;
    public double currentPos = 0;
    public static double close = 0.05;
    public static double open = 0.15;
    public static double initClaw = 0.3;
    public double currentClaw = 0;
    public static double closeClaw = 0.23;
    public static double looseClaw = 0.26;
    public static double openClaw = 0.4;
    public static double pickPos = 0.155;
    public static double placePos = 0.75;
    public static double autoReady = 0.72;
    public static double lowReadyPos = 0.78;
    public static double mediumReadyPos = 0.815;
    public static double distanceThreshold = 50;
    public boolean isOpen = false;
    public boolean isUp = false;
    public boolean isPlacing = false;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        rotateServo = hardwareMap.get(Servo.class, "goB1");
        clawServo = hardwareMap.get(Servo.class, "goB2");
        resetArm();
        resetClaw();
    }

    @Override
    public void periodic() {
        telemetry.addData("RotateServo", rotateServo.getPosition());
        telemetry.addData("ClawServo", clawServo.getPosition());
    }

    public void resetArm() {
        if(currentPos != initPos) {
            rotateServo.setPosition(initPos);
            isUp = true;
            currentPos = initPos;
        }
    }
    public void resetClaw() {
        if(currentClaw != initClaw) {
            clawServo.setPosition(initClaw);
            currentClaw = initClaw;
        }
    }

    public void open() {
        rotateServo.setPosition(open);
        isOpen = true;
    }

    public void close() {
        rotateServo.setPosition(close);
        isOpen = false;
    }

    public void openClaw() {
        clawServo.setPosition(openClaw);
        isOpen = true;
    }

    public void looseClaw() {
        clawServo.setPosition(looseClaw);
        isOpen = true;
    }

    public void closeClaw() {
        clawServo.setPosition(closeClaw);
        isOpen = false;
    }


}
