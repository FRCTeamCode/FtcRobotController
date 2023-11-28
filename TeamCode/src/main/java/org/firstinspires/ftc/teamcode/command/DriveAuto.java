package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.DrivePose;

public class DriveAuto extends CommandBase {

    private final DrivePose mDrive;
    //    private final MyCamera myCamera;
    private final Gamepad mGamepad;
    private ElapsedTime timer = new ElapsedTime();
    private final double driveKp = 0.3;

    public DriveAuto(DrivePose drive, Gamepad gamepad) {
        mDrive = drive;
        mGamepad = gamepad;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        timer.reset();
    }
    @Override
    public void execute() {
        mDrive.autoMoveXY(timer.milliseconds());
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}