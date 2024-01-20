package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;

public class RevieseDirec extends CommandBase {

//    private DrivePose drivePose;
    private double isRevise;

    public RevieseDirec(double isRevise) {
//        this.drivePose = drivePose;
        this.isRevise = isRevise;
    }

    @Override
    public void initialize() {
//        drivePose.resetIMU();
        AutoConstants.isUp = false;
        AutoConstants.isDown = false;
        AutoConstants.isOpRevise = isRevise;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
