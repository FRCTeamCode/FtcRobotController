package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;

public class ReverseUp extends CommandBase {
    private boolean isRevise;
    private boolean isDown;

    public ReverseUp(boolean isRevise, boolean isDown) {
        this.isRevise = isRevise;
        this.isDown = isDown;
    }

    @Override
    public void initialize() {
        AutoConstants.isUp = isRevise;
        if (isDown) {
            AutoConstants.isOpRevise = -1;
        } else {
            AutoConstants.isOpRevise = 1;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
