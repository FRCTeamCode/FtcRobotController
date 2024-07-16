package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
//    public Motor armMotor;
    public MotorEx arm2Motor;
    AnalogInput potentiometer;
    double currentVoltage;
    private final Telemetry telemetry;
    private double offsetPos = 0.477;
    private double mRotatePos = 0.575 - offsetPos;
    private double errorVel, xP, xI, xD, xLastError, vel;
    private double kXYP = 0.16, kXYI = 0.05, kXYD = 0.35, friction = 0.02;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        potentiometer = hardwareMap.get(AnalogInput.class, "pot");

//        armMotor = new Motor(hardwareMap, "motorArm", Motor.GoBILDA.RPM_84);
//        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        armMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.motor.setDirection(DcMotor.Direction.REVERSE);//REVERSE:1000->3000 arm up
//        armMotor.resetEncoder();
//        configPosition();//Pos:0-hold, 60-intake, 900-put

        arm2Motor = new MotorEx(hardwareMap, "right", Motor.GoBILDA.RPM_84);
        arm2Motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2Motor.setRunMode(Motor.RunMode.VelocityControl);
        arm2Motor.setVeloCoefficients(0.1, 0.015, 0.01);
        arm2Motor.setInverted(true);//0.577-low, 0.66-middle, 2.8-high//new:low-0.1
                                    //0.218-low, 0.66-middle, 2.8-high//new:low-0.1
    }
//    public void configPosition() {
//        armMotor.setRunMode(Motor.RunMode.PositionControl);
//        armMotor.setPositionCoefficient(0.013);
//        armMotor.setPositionTolerance(2.0);
//    }

    public void setArmPos(double pos) {
        mRotatePos = pos - offsetPos;
    }

    public void setVol(double vol) {
        arm2Motor.setVelocity(vol);
    }
    public  double getPotVol() {
        return currentVoltage;
    }

    public void pidReset() {
        xP = 0.0;
        xI = 0.0;
        xD = 0.0;
    }

    public void loop() {
        if (mRotatePos == -offsetPos) {
            arm2Motor.setVelocity(127.0);
        } else {
            currentVoltage = potentiometer.getVoltage();
            errorVel = mRotatePos - currentVoltage;
            xP = errorVel * kXYP;
            xI += errorVel;
            xI *= kXYI;
            xD = errorVel - xLastError;
            xD *= kXYD;
            vel = xP + xI + xD;
            vel = vel + addFriction(xP);
            xLastError = errorVel;
            vel = MathUtils.clamp(vel, -0.35, 0.4);
//            telemetry.addData("ArmPIDVelP", xP);
//            telemetry.addData("ArmPIDVelI", xI);
//            telemetry.addData("ArmPIDVelD", xD);
//            telemetry.addData("ArmPIDVel", vel);
            arm2Motor.setVelocity(vel*8000);
        }
    }

    public double addFriction(double value) {
        if (currentVoltage < 1.2) {
            return friction;
        } else
            if (currentVoltage > 2.0) {
            return -friction;
        } else {
            return 0;
        }
    }
    @Override
    public void periodic() {
        currentVoltage = potentiometer.getVoltage();
//        setSpeed(0.25);
//        telemetry.addData("ArmPos", arm2Motor.getCurrentPosition());
        loop();
//        currentVoltage = potentiometer.getVoltage();
        telemetry.addData("Potentiometer voltage", currentVoltage);
//        telemetry.addData("ArmVel", arm2Motor.getVelocity());
//        telemetry.addData("ArmCorVel", arm2Motor.getCorrectedVelocity());
    }
//    public void setSpeed(double speed) {
//        armMotor.set(speed);
//    }
//    public  void stopMotor() {
//        armMotor.stopMotor();
//    }

}
