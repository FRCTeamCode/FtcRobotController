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
        if (mClawPos == 1.0) {
            mClaw.lowClaw();
        } else if (mClawPos == 2.0) {
            mClaw.middleClaw();
        } else if (mClawPos == 3.0)  {
            mClaw.highClaw();
        } else if (mClawPos == 4.0) {
            mClaw.highLowerClaw();
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