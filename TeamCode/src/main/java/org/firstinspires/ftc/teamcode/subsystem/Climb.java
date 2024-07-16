package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climb extends SubsystemBase {
    public Motor climbMotor;
    private final Telemetry telemetry;
    private double mRotatePos = 0.575;

    public Climb(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climbMotor = new Motor(hardwareMap, "climb", Motor.GoBILDA.RPM_312);
        climbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        climbMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor.motor.setDirection(DcMotor.Direction.FORWARD);//REVERSE:1000->3000 arm up
//        climbMotor.resetEncoder();
        configPosition();//Pos:0-hold, 60-intake, 900-put
        setClimbPos(0);
    }
    public void configPosition() {
        climbMotor.setRunMode(Motor.RunMode.PositionControl);
        climbMotor.setPositionCoefficient(0.013);
        climbMotor.setPositionTolerance(2.0);
    }

    public void setClimbPos(int pos) {
        climbMotor.setTargetPosition(pos);
    }

    @Override
    public void periodic() {
        setSpeed(0.15);
        telemetry.addData("ClimbPos", climbMotor.getCurrentPosition());
    }
    public void setSpeed(double speed) {
        climbMotor.set(speed);
    }
    public  void stopMotor() {
        climbMotor.stopMotor();
    }

}
