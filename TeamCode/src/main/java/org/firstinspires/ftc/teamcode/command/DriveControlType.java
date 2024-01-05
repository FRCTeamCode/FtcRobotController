package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.drive.DrivePose;

public class DriveControlType extends CommandBase {

    private boolean isFieldControl;
    private DrivePose drivePose;

    public DriveControlType(Boolean isFieldControl, DrivePose drivePose) {
        this.isFieldControl =  isFieldControl;
        this.drivePose = drivePose;
    }

    @Override
    public void initialize() {
        AutoConstants.isFieldControl = isFieldControl;
        if (isFieldControl) {
            drivePose.resetIMU();
            AutoConstants.initAngle = 0.0;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
