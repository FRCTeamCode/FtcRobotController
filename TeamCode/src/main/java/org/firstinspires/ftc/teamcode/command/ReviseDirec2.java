package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.auton.AutoConstants;

public class ReviseDirec2 extends CommandBase {

    @Override
    public void initialize() {
        if (AutoConstants.isBlueOrRed) {
            AutoConstants.isOpRevise = -1;
        } else {
            AutoConstants.isOpRevise = -1;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
