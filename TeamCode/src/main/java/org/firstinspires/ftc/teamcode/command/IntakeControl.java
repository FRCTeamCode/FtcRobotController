package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class IntakeControl extends CommandBase {
    private Intake mIntake;
    private double mIntakePos;

    public IntakeControl(Intake intake, double intakePos) {
        mIntake = intake;
        mIntakePos =  intakePos;
        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        if (mIntakePos == 1.0) {
            mIntake.openIntake();
        } else if (mIntakePos == 2.0) {
            mIntake.looseIntake();
        } else if (mIntakePos == 3.0) {
            mIntake.closeIntake();
        } else if (mIntakePos == 4.0) {
            mIntake.intakeSetPos(0.282);
        } else if (mIntakePos == 5.0) {
            mIntake.intakeSetPos(0.302);
        } else if (mIntakePos == 6.0) {
            mIntake.intakeSetPos(0.322);
        } else if (mIntakePos == 7.0) {
            mIntake.intakeSetPos(0.352);
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