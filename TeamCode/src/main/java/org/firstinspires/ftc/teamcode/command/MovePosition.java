package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystem.Climb;

public class MovePosition extends CommandBase {
    private final Climb climb;
    private final int mPosition;
    public MovePosition(Climb testMotor, int position) {
        this.climb = testMotor;
        mPosition = position;
        addRequirements(testMotor);
    }

    @Override
    public void initialize() {
        climb.setClimbPos(mPosition);
        if (Math.abs(mPosition)>2000) {
            AutoConstants.isOpRevise = -1;
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
