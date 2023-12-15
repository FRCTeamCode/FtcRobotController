package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;

public class DriveControlType extends CommandBase {

    private boolean isFieldControl;

    public DriveControlType(Boolean isFieldControl) {
        this.isFieldControl =  isFieldControl;
    }

    @Override
    public void initialize() {
        AutoConstants.isFieldControl = isFieldControl;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
