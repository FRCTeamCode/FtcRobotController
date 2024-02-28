package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;

public class EleControl extends CommandBase {
    private Elevator mEle;
    private double mClawPos;

    public EleControl(Elevator ele, double elePos) {
        mEle = ele;
        mClawPos  = elePos;
        addRequirements(mEle);
    }

    @Override
    public void initialize() {
        if (mClawPos == 0.0) {
            mEle.defaultEle();
        }else if (mClawPos == 1.0) {
            mEle.middleEle();
        } else if (mClawPos == 2.0) {
            mEle.highEle();
        } else if (mClawPos == 3.0) {
            mEle.eleIntake();
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