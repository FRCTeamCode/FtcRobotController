package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;

public class ReverseUp extends CommandBase {
    private boolean isRevise;

    public ReverseUp(boolean isRevise) {
        this.isRevise = isRevise;
    }

    @Override
    public void initialize() {
        AutoConstants.isUp = isRevise;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
