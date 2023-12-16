package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.DrivePose;

public class IMUReset extends CommandBase {

    private DrivePose drivePose;

    public IMUReset(DrivePose drivePose) {
        this.drivePose = drivePose;
    }

    @Override
    public void initialize() {
        drivePose.resetIMU();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
