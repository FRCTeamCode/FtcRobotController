package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;

public class ClawControl extends CommandBase {
    private Claw mClaw;
    private double mClawPos;

    public ClawControl(Claw claw, double clawPos) {
        mClaw = claw;
        mClawPos  = clawPos;
        addRequirements(mClaw);
    }

    @Override
    public void initialize() {
        if (mClawPos == 0.0) {
            mClaw.lowerClaw();
        }else if (mClawPos == 1.0) {
            mClaw.lowClaw();
        } else if (mClawPos == 2.0) {
            mClaw.middleClaw();
        } else if (mClawPos == 3.0)  {
            mClaw.highClaw();
        } else if (mClawPos == 4.0) {
            mClaw.highLowerClaw();
        } else if (mClawPos == 6.0) {
            mClaw.middleLowClaw();
        } else if (mClawPos == 7.0) {
            mClaw.pixelPutLowClaw();
        } else if (mClawPos == 8.0) {
            mClaw.pixelPutMidClaw();
        } else if (mClawPos == 9.0) {
            mClaw.pixelPutHigherClaw();
        } else if (mClawPos == 10.0) {
            mClaw.pixelPutHighestClaw();
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}