package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.drive.DrivePose;

public class RedTeleopDrive extends CommandBase {

    private final DrivePose mDrive;
    //    private final MyCamera myCamera;
    private final Gamepad mGamepad;
    private ElapsedTime timer = new ElapsedTime();
    private final  double driveKp = 0.3;
    public RedTeleopDrive(DrivePose drive, Gamepad gamepad) {
        mDrive = drive;
        mGamepad = gamepad;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
//        mDrive.mecanumFieldDrive(mGamepad.left_stick_x * driveKp
//                , mGamepad.left_stick_y * driveKp
//                , mGamepad.right_stick_x * driveKp, false);
//        mDrive.mecanumCentricDrive(mGamepad.left_stick_x * driveKp
//                                 , mGamepad.left_stick_y * driveKp
//                                 , mGamepad.right_stick_x * driveKp, false);
//        mDrive.driveJoy(mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75),
//                mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75),
//                mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.5);
//
        if (AutoConstants.isUp) {
            mDrive.mecanumCentricDrive(mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75),
                    -mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75),
                    -mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.5, 1.0);
//            mDrive.driveField(
//                    mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75),
//                    mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75),
//                    mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.65);
        } else{
            mDrive.mecanumCentricDrive(
                mGamepad.left_stick_x * (1.0 - mGamepad.left_trigger * 0.75) * AutoConstants.isOpRevise,
                mGamepad.left_stick_y * (1.0 - mGamepad.left_trigger * 0.75) * AutoConstants.isOpRevise,
                -mGamepad.right_stick_x * (1.0 - mGamepad.left_trigger * 0.75)*0.5, 1.0);
        }

//        mDrive.driveJoy(-mGamepad.left_stick_y, mGamepad.left_stick_x, mGamepad.right_stick_x);//normal drive

//        mDrive.driveField(mGamepad.left_stick_y, mGamepad.left_stick_x, mGamepad.right_stick_x);

//        mDrive.autoMoveXY(timer.milliseconds());

//        double[] id = myCamera.getAprilTagIDData(10);
//        mDrive.driveAlign(id[0], id[1], id[2], id[3]);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}