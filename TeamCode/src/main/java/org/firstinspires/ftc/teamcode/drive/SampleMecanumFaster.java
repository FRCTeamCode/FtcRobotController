package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.util.MathUtils;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.hardware.NavxMicro;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import java.text.DecimalFormat;

public class SampleMecanumFaster extends MecanumDrive {
    private StandardTrackingWheelLocalizer sTWLoclizer;
    private BHI260IMU imu;
//    private final AHRS navx2micro;
    private Pose2d poseEstimate;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6.5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Telemetry telemetry;
    private List<DcMotorEx> motors;
    private VoltageSensor batteryVoltageSensor;
    DecimalFormat df = new DecimalFormat("#.##");
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private boolean calibration_complete = false;
    double pastv = 0;
    double pastv1 = 0;
    double pastv2 = 0;
    double pastv3 = 0;
    private List<Double> tWheelPos;
//    private double strafe = 9.0, translation = 0.0, rotation = -1.05;
    private double translationVal, strafeVal, rotationVal, translationFriction, strafeFriction, rotationFriction;
    private boolean  isHasTarget, tolerance, standStill, isHasFindTarget;

    public SampleMecanumFaster(HardwareMap hardwareMap, Telemetry telemetry) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.1, 0.1, Math.toRadians(0.5)), 0.3);
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        this.telemetry = telemetry;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        if (!AutoConstants.isInitNavxMicro2) {
            NavxMicro navxMicro = new NavxMicro(hardwareMap, telemetry);
        }

//        navx2micro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
//                AHRS.DeviceDataType.kProcessedData,
//                NAVX_DEVICE_UPDATE_RATE_HZ);
//
//        while ( !calibration_complete ) {
//            /* navX-Micro Calibration completes automatically ~15 seconds after it is
//            powered on, as long as the device is still.  To handle the case where the
//            navX-Micro has not been able to calibrate successfully, hold off using
//            the navX-Micro Yaw value until calibration is complete.
//             */
//            calibration_complete = !navx2micro.isCalibrating();
//            if (!calibration_complete) {
//                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
//            }
//        }
//        navx2micro.zeroYaw();


        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
//        rightRear = hardwareMap.get(DcMotorEx.class, "rightFront");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightRear");
        //Normal drive use rear camera track Tag
        leftFront = hardwareMap.get(DcMotorEx.class, "motorTest0");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorTest1");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorTest2");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorTest3");
        //Rotate drive use rear camera track Tag
//        leftFront = hardwareMap.get(DcMotorEx.class, "motorTest3");
//        leftRear = hardwareMap.get(DcMotorEx.class, "motorTest0");
//        rightRear = hardwareMap.get(DcMotorEx.class, "motorTest1");
//        rightFront = hardwareMap.get(DcMotorEx.class, "motorTest2");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        sTWLoclizer = new StandardTrackingWheelLocalizer(hardwareMap);
        setLocalizer(sTWLoclizer);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public void initAuto() {
        tolerance = false;
    }

    public void autoMoveXY(double xDist, double xK, double yDist, double yK, double runTime, double timeout, double brakeDist, double speedLimit) {
        tWheelPos = sTWLoclizer.getWheelPos();//Localizer wheel: [0]-left, [1]-right, [2]-front
        double strafeVal = 0, xError = 0, yError = 0, zError = 0.0, translationVal = 0.0;
        //move forward and backward -------------------------------------------//
        xError = xDist - tWheelPos.get(1);
        if (Math.abs(xError) < brakeDist && runTime > 500.0) {//Start decrease speed and stop
            strafeVal = MathUtils.clamp(xError/brakeDist*xK*0.6, -speedLimit, speedLimit);
        } else {//Increase speed until speedLimit
            strafeVal = MathUtils.clamp( Math.signum(xError) * (runTime/500.0) * xK, -speedLimit, speedLimit);
        }
        double strafeFriction = Math.abs(xError) < 0.02 ? 0.0 : Math.signum(strafeVal) * 0.055;
        //Move left and right -------------------------------------------------//
        yError = yDist - tWheelPos.get(2);
        if (Math.abs(yError) < brakeDist) {//Start decrease speed and stop
            translationVal = MathUtils.clamp(yError/brakeDist*yK, -speedLimit, speedLimit);
        } else {//Increase speed until speedLimit
            translationVal = MathUtils.clamp( Math.signum(yError) * runTime/500.0 * yK, -speedLimit, speedLimit);
        }
        double transFriction = Math.abs(yError) < 0.02 ? 0.0 : Math.signum(translationVal) * 0.055;
        //move rotate ---------------------------------------------------------//
//        zError = tWheelPos.get(0)-tWheelPos.get(1);
        zError = Math.toDegrees(getRawExternalHeading());
        double rotationVal = MathUtils.clamp(zError * 0.05, -0.1, 0.1);
        double rotationFriction = Math.abs(zError) < 0.02 ? 0.0 : Math.signum(rotationVal) * 0.03;
        if (runTime < timeout) {
//            updateRobotDrive(
//                    -strafeVal + strafeFriction,
//                    translationVal + transFriction,
//                    rotationVal + rotationFriction,
//                    1.0
//            );
            updateRobotDrive(
                    -strafeVal - strafeFriction,
                    translationVal + transFriction,
                    rotationVal + rotationFriction,
                    1.0
            );
//            setWeightedDrivePower(
//                    new Pose2d(
//                            strafeVal + strafeFriction,
//                            translationVal + transFriction,
//                            rotationVal + rotationFriction
//                    )
//            );
        } else {
            updateRobotDrive(0.0,0.0,0.0,0.0);
//            setWeightedDrivePower(new Pose2d(0.0, 0.0, 0.0));
        }
        tolerance = runTime > 100 && (Math.abs(strafeVal) < 0.01) && (Math.abs(translationVal) < 0.02) && (Math.abs(rotationVal) < 0.015);
        telemetry.addData("Error1", strafeVal);
        telemetry.addData("Error2", translationVal);
        telemetry.addData("Error3", rotationVal);
        telemetry.addData("Runtime", runTime);
    }

    public boolean isEndAutoMove() {
        return tolerance;
    }
    public boolean isHasTarget() {
        return isHasTarget;
    }

    public void initTagPara() {
        tolerance = false;
        isHasFindTarget = false;
    }

    public boolean isEndAlign() {
        return tolerance;
    }

    public boolean isHasFindTarget() {
        return isHasFindTarget;
    }

    public void alinTag(double[] tag, int targetID, double strafe, double translation, double rotation, double time) {
        if (tag[0] == 0) {
            if (tag[0] != -1.0) {
                isHasTarget = true;
            } else {
                isHasTarget = false;
            }
        } else {
            if (tag[0] == targetID) {
                isHasTarget = true;
            } else {
                isHasTarget = false;
            }
        }
        if (isHasTarget) {
            if (!isHasFindTarget) {
                isHasFindTarget = true;
            }
            //move forward and backward
            if (tag[1] - strafe < 30.0) {
                strafeVal = MathUtils.clamp((tag[1] - strafe) * 0.1, -0.4, 0.4);
                strafeFriction = Math.signum(strafeVal) * 0.009;
            } else {
                strafeVal = 0.0;
            }
            //Move left and right
            double dist = Math.abs(tag[1] - strafe);
            double kp2 = dist > 4.0 ? dist / 3.0 * dist / 3.0 : 1.0;
            translationVal = MathUtils.clamp((tag[2] - translation) * 0.35 * kp2, -0.32, 0.32);
            translationFriction = Math.signum(translationVal) * 0.0055;
            //move rotate
            double kp3 = dist > 4.0 ? 1.3 : 1.0;
            rotationVal = MathUtils.clamp((rotation - tag[3]) * 0.2 * kp3, -0.13, 0.13);
            rotationFriction = Math.signum(rotationVal) * 0.008;
            updateRobotDrive(
                    (strafeVal * 0.5 + strafeFriction),             //forward and backward
                    -(translationVal * 0.5 + translationFriction),  //left and right
                    -(rotationVal * 0.5 + rotationFriction),         //rotate left and right
                    1.0
            );
            telemetry.addData("Error1", strafeVal);
            telemetry.addData("Error2", translationVal);
            telemetry.addData("Error3", rotationVal);
            if ((Math.abs(strafeVal) < 0.14) && (Math.abs(translationVal) < 0.14) && (Math.abs(rotationVal) < 0.1)) {
                tolerance = true;
            } else {
                tolerance = false;
            }
        } else {
//            if (isHasFindTarget) {
//                tolerance = true;
//            }
            updateRobotDrive(0.0,0.0,0.0,1.0);
        }
        telemetry.addData("Auto is End Align", isEndAlign());
    }

    public void alignAprilTag(double x, double y, double r, double id, double X, double Y, double R) {
        double strafeVal = 0.0, strafeFriction;
        //Move left and right
        double dist = Math.abs(X - x);

        double kp2 = dist > 5.0 ? dist / 5.0 : 1.0;
        double errorY = MathUtils.clamp(y - Y, -1.0, 1.0);
        errorY = Math.abs(errorY) > 0.25 ? errorY * Math.abs(errorY) : errorY;
        double translationVal = MathUtils.clamp(errorY * 0.3 * kp2, -0.32, 0.32);
        double translationFriction = Math.abs(translationVal) < 0.01 ? 0.0 :  Math.signum(translationVal) * 0.04;
        //move rotate
        double kp3 = dist > 4.0 ? 1.3 : 1.0;
        double rotationVal = -MathUtils.clamp((R - r) * 0.2 * kp3, -0.13, 0.13);
        double rotationFriction = Math.abs(rotationVal) < 0.01 ? 0.0 :  Math.signum(rotationVal) * 0.012;
        //move forward and backward
        if (Math.abs(errorY) < 0.3 && Math.abs(rotationVal) < 0.08) {
            strafeVal = MathUtils.clamp((X - x) * 0.1, -0.4, 0.4);
            strafeFriction = Math.abs(strafeVal) < 0.01 ? 0.0 :  Math.signum(strafeVal) * 0.035;
        } else {
            strafeFriction = 0.0;
        }
        telemetry.addData("Error1-front-back", strafeVal);
        telemetry.addData("Error2-left-right", translationVal);
        telemetry.addData("Error3-rotate", rotationVal);
        tolerance = (Math.abs(strafeVal) < 0.02) && (Math.abs(translationVal) < 0.033) && (Math.abs(rotationVal) < 0.024);
//        if (id != 0) {
//            setWeightedDrivePower(
//                    new Pose2d(
//                            strafeVal * 0.4 + strafeFriction,
//                            translationVal * 0.4 + translationFriction,
//                            (rotationVal * 0.4 + rotationFriction)
//                    )
//            );
//        } else {
//            setWeightedDrivePower(new Pose2d(0.0, 0.0, 0.0));
//        }
    }

    public void update() {
//        telemetry.addData("left", sTWLoclizer.getWheelPos().get(0));
//        telemetry.addData("right", sTWLoclizer.getWheelPos().get(1));
//        telemetry.addData("front", sTWLoclizer.getWheelPos().get(2));
        telemetry.addData("ImuHeading", Math.toDegrees(getRawExternalHeading()));
//        telemetry.addData("NavxYaw", df.format(navx2micro.getYaw()));
        telemetry.update();
        updatePoseEstimate();
        poseEstimate = getPoseEstimate();
        telemetry.addData("TWLPoseX", poseEstimate.getX());
        telemetry.addData("TWLPoseY", poseEstimate.getY());
        telemetry.addData("TWLPoseH", Math.toDegrees(poseEstimate.getHeading()));
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public Pose2d getMyPose() {
        return poseEstimate;
    }

    public void updateRobotDrive(double left_stick_y, double left_stick_x, double right_stick_x, double driveK) {
        double y = -left_stick_y;
        double x = left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = right_stick_x;
        // Denominator is the largest motor power (absolute value) or 1
        // Keeps all motor powers in proportion
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator * driveK;
        double backLeftPower = (y - x + rx) / denominator * driveK;
        double frontRightPower = (y - x - rx) / denominator * driveK;
        double backRightPower = (y + x - rx) / denominator * driveK;
        // Set powers
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
        telemetry.addData("DriveType", AutoConstants.isFieldControl);
    }
    public void updateField(double left_stick_y, double left_stick_x, double right_stick_x, double driveK) {
        double y = -left_stick_y;
        double x = left_stick_x * 1.1;
        double rx = right_stick_x;

//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - AutoConstants.initAngle;
//        double botHeading = getRawExternalHeading() - AutoConstants.initAngle;
//        double botHeading = AngleUnit.DEGREES.toRadians(-navxMicro.getYaw()) - AutoConstants.initAngle;
        double botHeading = AngleUnit.DEGREES.toRadians(-NavxMicro.getYaw()) - AutoConstants.initAngle;

        // Rotate the movement direction counter to the robot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (rotY + rotX + rx) / denominator * driveK;
        double backLeftPower = (rotY - rotX + rx) / denominator * driveK;
        double frontRightPower = (rotY - rotX - rx) / denominator * driveK;
        double backRightPower = (rotY + rotX - rx) / denominator * driveK;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

//        telemetry.addData("Heading", AngleUnit.RADIANS.toDegrees(getPoseEstimate().getHeading()));
        telemetry.addData("HeadingPos", AngleUnit.RADIANS.toDegrees(botHeading));
//        telemetry.addData("DriveType", AutoConstants.isFieldControl);
    }

    public void resetIMU() {
        imu.resetYaw();
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        pastv = v;

        leftRear.setPower(v1);
        pastv1 = v1;

        rightRear.setPower(v2);
        pastv2 = v2;
        rightFront.setPower(v3);
        pastv3 = v3;

    }

    @Override
    public double getRawExternalHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public double getImuRad() {
        return getRawExternalHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return (double) angularVelocity.zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
