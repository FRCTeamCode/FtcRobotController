package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

public class ArmControl extends CommandBase {
    private Arm mArm;
    private  int mRotatePos;
    public ArmControl(Arm arm, int rotatePos) {
        mArm = arm;
        mRotatePos = rotatePos;
        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mArm.setArmPos(mRotatePos);
    }

    @Override
    public void execute() {
//        mArm.setSpeed(0.13);
    }

    @Override
    public void end(boolean interrupted) {
//        mArm.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
