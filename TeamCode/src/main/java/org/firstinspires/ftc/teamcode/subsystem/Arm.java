package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    public Motor armMotor;
    private final Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = new Motor(hardwareMap, "motorArm", Motor.GoBILDA.RPM_84);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.motor.setDirection(DcMotor.Direction.REVERSE);//REVERSE:1000->3000 arm up
        armMotor.resetEncoder();
        configPosition();//Pos:0-hold, 60-intake, 900-put
    }
    public void configPosition() {
        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setPositionCoefficient(0.013);
        armMotor.setPositionTolerance(2.0);
    }

    @Override
    public void periodic() {
        setSpeed(0.14);
        telemetry.addData("ArmPos", armMotor.getCurrentPosition());
    }
    public void setSpeed(double speed) {
        armMotor.set(speed);
    }

    public void setArmPos(int pos) {
        armMotor.setTargetPosition(pos);
    }
    public  void stopMotor() {
        armMotor.stopMotor();
    }

}
