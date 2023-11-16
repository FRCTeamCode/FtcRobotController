package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

public class ArmControl extends CommandBase {
    private Arm mArm;
    private  double mRotatePos;
    public ArmControl(Arm arm, double rotatePos) {
        mArm = arm;
        mRotatePos = rotatePos;
        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        if (mRotatePos==1.0) {
            mArm.open();
            mArm.openClaw();
        } else if (mRotatePos==2.0) {
            mArm.looseClaw();
        } else {
            mArm.close();
            mArm.closeClaw();
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
