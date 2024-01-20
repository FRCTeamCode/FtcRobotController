package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;

public class ReverseUp extends CommandBase {

    //    private DrivePose drivePose;
    private boolean isRevise;
    private boolean isDown;

    public ReverseUp(boolean isRevise, boolean isDown) {
//        this.drivePose = drivePose;
        this.isRevise = isRevise;
        this.isDown = isDown;
    }

    @Override
    public void initialize() {
//        drivePose.resetIMU();
        if (isDown) {
            AutoConstants.isDown = isRevise;
            AutoConstants.isUp = false;
            AutoConstants.isOpRevise = -1;
        } else {
            AutoConstants.isUp = isRevise;
            AutoConstants.isDown = false;
            AutoConstants.isOpRevise = 1;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
