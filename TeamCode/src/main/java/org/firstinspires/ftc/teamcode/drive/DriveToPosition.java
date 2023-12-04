package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;

public class DriveToPosition  extends CommandBase {

    private final DrivePose mDrive;
    //    private final MyCamera myCamera;
    private final Gamepad mGamepad;
    private ElapsedTime timer = new ElapsedTime();
    private final  double driveKp = 0.3;
    public DriveToPosition (DrivePose drive, Gamepad gamepad) {
        mDrive = drive;
        mGamepad = gamepad;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d pose = mDrive.getPoseEstimate();

//        mDrive.mecanumFieldDrive(mGamepad.left_stick_x * driveKp
//                , mGamepad.left_stick_y * driveKp
//                , mGamepad.right_stick_x * driveKp, false);
//        mDrive.mecanumCentricDrive(mGamepad.left_stick_x * driveKp
//                                 , mGamepad.left_stick_y * driveKp
//                                 , mGamepad.right_stick_x * driveKp, false);
//        mDrive.driveJoy(mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75),
//                mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75),
//                mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.5);
//        if (Math.abs(mGamepad.right_trigger) < 0.5) {
//            mDrive.driveField(-mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75),
//                    mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75),
//                    mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.5);
//        } else{
        mDrive.mecanumCentricDrive(-mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75),
                -mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75),
                -mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.5, 1.0);

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}