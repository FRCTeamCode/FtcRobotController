package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends SubsystemBase {
    public Servo elevatorServo;
    private final Telemetry telemetry;
    public static double defaultEle = 0.185;
    public static double eleIntake = 0.26-(0.26-0.185)/2;
    public static double eleIntakePro = 0.29-(0.26-0.185)/2;
    public static double autoPixel5Ele = 0.36-(0.36-0.185)/2;
    public static double middleEle = 0.43-(0.43-0.185)/2;
    public static double highEle = 0.635-(0.62-0.185)/2;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorServo = hardwareMap.get(Servo.class, "goB3");//0.23->0.4 intake open
        elevatorServo.setDirection(Servo.Direction.FORWARD);
        defaultEle();
    }

    @Override
    public void periodic() {
        telemetry.addData("ElevatorServo", elevatorServo.getPosition());
    }
    public  void  intakeSetPos(double pos) {
        elevatorServo.setPosition(pos);
    }

    public void highEle() {
        elevatorServo.setPosition(highEle);
    }

    public void middleEle() {
        elevatorServo.setPosition(middleEle);
    }
    public void autoPixel5Ele() {
        elevatorServo.setPosition(autoPixel5Ele);
    }

    public void defaultEle() {
        elevatorServo.setPosition(defaultEle);
    }
    public void eleIntake() {
        elevatorServo.setPosition(eleIntake);
    }
    public void eleIntakePro() {
        elevatorServo.setPosition(eleIntakePro);
    }
    public double getPosition() {
        return elevatorServo.getPosition();
    }
}
