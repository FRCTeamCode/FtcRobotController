package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.CameraAuto;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class AutoBlueLeft extends LinearOpMode {
    SampleMecanumFaster drive;
//    MyCamera myCamera;
    CameraAuto cameraAuto;
    Arm arm;
    Claw claw;
    Intake intake;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    AprilTagDetection tagOfInterest = null;
    String targetSide = "";
    TrajectorySequence targetRoad;
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;
    double currentTime = 0;
    private int tagID = 4;
    public static double cycleDelay = .52;
    public static double turretAfterScoreDelay = .7;
    private boolean isAutoEnd = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumFaster(hardwareMap, dashboardTelemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(AutoConstants.START);

        arm = new Arm(hardwareMap, dashboardTelemetry);
        claw = new Claw(hardwareMap, dashboardTelemetry);
        intake = new Intake(hardwareMap, dashboardTelemetry);
//        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);
        cameraAuto = new CameraAuto(hardwareMap, dashboardTelemetry);

        isAutoEnd = false;

        TrajectorySequence pathLeft = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0.0, () -> {
                    arm.setArmPos(1.08);
                })
                .addTemporalMarker(0.9, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(1.6, () -> {
                    intake.openIntake();
                })
                .lineToLinearHeading(AutoConstants.BL1_PUT)
                .addTemporalMarker(3.0, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.0, () -> {
                    arm.setArmPos(2.36);
                })
                .addTemporalMarker(3.6, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BL1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    arm.setArmPos(3.1);
                })
                .waitSeconds(1.5)
                .lineToLinearHeading(AutoConstants.BL1_Tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    arm.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    arm.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BL1_STOP)
                .build();

        TrajectorySequence pathMiddle = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0.0, () -> {
                    arm.setArmPos(1.08);
                })
                .addTemporalMarker(2.0, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.8, () -> {
                    intake.openIntake();
                })
                .lineToLinearHeading(AutoConstants.BM1_PUT)
                .addTemporalMarker(3.6, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.6, () -> {
                    arm.setArmPos(2.36);
                })
                .addTemporalMarker(4.2, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BM1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    arm.setArmPos(3.1);
                })
                .waitSeconds(1.5)
                .lineToLinearHeading(AutoConstants.BM1_Tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    arm.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    arm.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BM1_STOP)
                .build();
        TrajectorySequence pathRight = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0.0, () -> {
                    arm.setArmPos(1.08);
                })
                .addTemporalMarker(1.6, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.1, () -> {
                    intake.openIntake();
                })
                .lineToLinearHeading(AutoConstants.BR1_PUT)
                .addTemporalMarker(3.0, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.0, () -> {
                    arm.setArmPos(2.25);
                })
                .addTemporalMarker(3.5, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BR1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    arm.setArmPos(3.1);
                })
                .waitSeconds(1.5)
                .lineToLinearHeading(AutoConstants.BR1_Tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    arm.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    arm.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BR1_STOP)
                .build();

        TrajectorySequence pathLeftPark = drive.trajectorySequenceBuilder(AutoConstants.BL1_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
//                .addTemporalMarker(0, () -> {
//                    turret.setTargetAngle(100);
//                })
                .lineToLinearHeading(AutoConstants.BL1_STOP)
//                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(300);
//                })
                .waitSeconds(1.5)
//                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .build();
        TrajectorySequence pathMiddlePark = drive.trajectorySequenceBuilder(AutoConstants.BM1_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
//                .addTemporalMarker(0, () -> {
//                    turret.setTargetAngle(100);
//                })
                .lineToLinearHeading(AutoConstants.BM1_STOP)
//                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(300);
//                })
                .waitSeconds(1.5)
//                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .build();
        TrajectorySequence pathRightPark = drive.trajectorySequenceBuilder(AutoConstants.BR1_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
//                .addTemporalMarker(0, () -> {
//                    turret.setTargetAngle(100);
//                })
                .lineToLinearHeading(AutoConstants.BR1_STOP)
//                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(300);
//                })
                .waitSeconds(1.5)
//                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .build();

        targetRoad = pathRight;

        while (!isStarted() && !isStopRequested()) {
//            myCamera.runDoubleVision();
            cameraAuto.runDoubleVision();
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
            dashboardTelemetry.addData("minT", dashboardTelemetry.getMsTransmissionInterval());

            currentTime = timer.milliseconds();
            dashboardTelemetry.addLine("run-time: " + currentTime/1000);

            List<Recognition> currentRecognitions = cameraAuto.getTfodData();
            if (currentRecognitions.size() != 0) {
                boolean tagFound = false;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//                    dashboardTelemetry.addData(""," ");
//                    dashboardTelemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                    dashboardTelemetry.addData("- Position", "%.0f / %.0f", x, y);
//                    dashboardTelemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    if (recognition.getLabel() == "BlueCube") {
                        tagFound = true;
                        if (x < 280 && x > 80 && Math.abs(y-135) < 40 && Math.abs(recognition.getWidth()-87) < 20 && Math.abs(recognition.getHeight()-95) < 20) {
                            tagID = 4;
                            targetSide = "Left";
                            targetRoad = pathLeft;
                        } else if (x > 280 && Math.abs(y-115) < 40 && Math.abs(recognition.getWidth()-75) < 20 && Math.abs(recognition.getHeight()-88) < 20) {
                            tagID = 5;
                            targetSide = "Middle";
                            targetRoad = pathMiddle;
                        } else {
                            tagID = 6;
                            targetSide = "Right";
                            targetRoad = pathRight;
                        }
                    }
                }

                if (tagFound) {
                    dashboardTelemetry.addLine("Tag of interest is in sight!\n\nLocation data: " + targetSide);
                } else {
                    dashboardTelemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        dashboardTelemetry.addLine("(The tag has never been seen1)");
                    } else {
                        dashboardTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                    }
                }

            } else {
                dashboardTelemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    dashboardTelemetry.addLine("(The tag has never been seen2)");
                } else {
                    dashboardTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                }

            }
            dashboardTelemetry.update();
            sleep(20);
        }
        drive.followTrajectorySequenceAsync(targetRoad);
        lastTime = timer.milliseconds();

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            arm.loop();
//            myCamera.runDoubleVision();

//            if (!drive.isBusy()&&!isAutoEnd) {
//                //path has finished
//                double[] idData = myCamera.getAprilTagIDData(tagID);
//                drive.alignAprilTag(10.0, 0.0, 0.0, idData[0], idData[1], idData[2], idData[3]);
//                dashboardTelemetry.addLine("AprilTag Tracking");
//                if (drive.isEndAlign()) {
//                    isAutoEnd = true;
//                    if (targetSide == "Left") {
//                        drive.followTrajectorySequenceAsync(pathLeftPark);
//                    } else if (targetSide == "Middle") {
//                        drive.followTrajectorySequenceAsync(pathMiddlePark);
//                    } else {
//                        drive.followTrajectorySequenceAsync(pathRightPark);
//                    }
//                }
//
//            }
            currentTime = timer.milliseconds();
            dashboardTelemetry.addLine("run-time: " + currentTime/1000);
            dashboardTelemetry.addLine("loop-time: " + (currentTime - lastTime)/1000);

            lastTime = currentTime;
//            dashboardTelemetry.update();
        }

    }
}
