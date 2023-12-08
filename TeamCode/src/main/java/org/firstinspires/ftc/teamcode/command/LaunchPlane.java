package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Launch;

public class LaunchPlane extends CommandBase {
    private Launch mLaunch;
    private double mLaunchPos;

    public LaunchPlane(Launch claw, double launchPos) {
        mLaunch = claw;
        mLaunchPos = launchPos;
        addRequirements(mLaunch);
    }

    @Override
    public void initialize() {
        if (mLaunchPos == 1.0) {
            mLaunch.lockPos();
        } else if (mLaunchPos == 2.0) {
            mLaunch.launchPos();
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