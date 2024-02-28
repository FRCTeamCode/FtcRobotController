package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;

public class EleIsBack extends CommandBase {
    private Elevator mEle;

    public EleIsBack(Elevator ele) {
        mEle = ele;
        addRequirements(mEle);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return mEle.getPosition() < 0.25;
    }
}