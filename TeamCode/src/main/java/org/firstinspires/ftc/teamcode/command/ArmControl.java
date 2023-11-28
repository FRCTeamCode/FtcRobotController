package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

public class ArmControl extends CommandBase {
    private Arm mArm;
    private  double mPotPos;
    private double errorVel, xP, xI, xD, xLastError, vel;
    private double x, y, r, kXYP = 0.5, kXYI = 0.2, kXYD = 0.2;

    public ArmControl(Arm arm, double potPos) {
        mArm = arm;
        mPotPos = potPos;
        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mArm.pidReset();
        mArm.setArmPos(mPotPos);
//        mRotatePos = MathUtils.clamp(mRotatePos, 0.58, 2.99);
    }

    @Override
    public void execute() {
//        mArm.setSpeed(0.13);

//        errorVel = mRotatePos - mArm.getPotVol();
//        xP = errorVel * kXYP;
//        xI += errorVel;
//        xI *= kXYI;
//        xD = errorVel - xLastError;
//        xD *= kXYD;
//        vel = xP + xI + xD;
//        vel = MathUtils.clamp(vel, -0.8, 1.2);
//        xLastError = errorVel;
//        mArm.setVol(MathUtils.clamp(errorVel * 3600, -800,1200));
    }

    @Override
    public void end(boolean interrupted) {
//        mArm.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
