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
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class AutoBlueMiddle extends LinearOpMode {
    SampleMecanumFaster drive;
    MyCamera myCamera;
    Arm arm;
    Claw claw;
    Intake intake;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    AprilTagDetection tagOfInterest = null;
    String targetSide = "";
    TrajectorySequence targetRoad;
    private enum Auto_State {
        moveOneSide, moveBackstage, detectAprilTag, park, stop
    }
    private enum Command_State {
        init, execute, end
    }
    private Auto_State aState = Auto_State.moveOneSide;
    private Command_State cState = Command_State.init;
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;
    double currentTime = 0;
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
        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);

        isAutoEnd = false;

//        turret = new Turret();
//        turret.init(hardwareMap, dashboardTelemetry);
//        turret.setTargetAngle(0);

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
                .lineToLinearHeading(AutoConstants.BL1_STOP)
                .addTemporalMarker(4.5, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(5.5, () -> {
                    claw.middleClaw();
                })
//                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(300);
//                })
//                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .waitSeconds(1.5)
                .lineToLinearHeading(AutoConstants.BL1_BACKSTAGE)
                .build();

        TrajectorySequence pathMiddle = drive.trajectorySequenceBuilder(AutoConstants.START)
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
        TrajectorySequence pathRight = drive.trajectorySequenceBuilder(AutoConstants.START)
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

        targetRoad = pathLeft;

        while (!isStarted() && !isStopRequested()) {
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
            dashboardTelemetry.addData("minT", dashboardTelemetry.getMsTransmissionInterval());

            List<Recognition> currentRecognitions = myCamera.getTfodData();
            if (currentRecognitions.size() != 0) {
                boolean tagFound = false;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    if (recognition.getLabel() == "Cube") {
                        tagFound = true;
                        if (x < 50 && y < 50) {
                            targetSide = "Left";
                            targetRoad = pathLeft;
                        } else if (x < 50 && y < 50) {
                            targetSide = "Middle";
                            targetRoad = pathMiddle;
                        } else {
                            targetSide = "Right";
                            targetRoad = pathRight;
                        }
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data: " + targetSide);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                }

            }
            sleep(20);
        }
        lastTime = timer.milliseconds();

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            arm.loop();
            if (!isAutoEnd) {
//                if (targetSide == "Left") {
//                    drive.followTrajectorySequenceAsync(pathLeft);
//                } else if (targetSide == "Middle") {
//                    drive.followTrajectorySequenceAsync(pathMiddle);
//                } else {
//                    drive.followTrajectorySequenceAsync(pathRight);
//                }
                drive.followTrajectorySequenceAsync(targetRoad);
                isAutoEnd = true;
            }
//
//            switch (aState) {
//                case moveOneSide:
//                    dashboardTelemetry.addLine("move one side updating" + targetSide);
////                    if (targetSide == "Left") {
////                        drive.followTrajectorySequenceAsync(pathLeft);
////                    } else if (targetSide == "Middle") {
////                        drive.followTrajectorySequenceAsync(pathMiddle);
////                    } else {
////                        drive.followTrajectorySequenceAsync(pathRight);
////                    }
//                    drive.followTrajectorySequenceAsync(targetRoad);
//                    aState = Auto_State.moveBackstage;
//                    break;
//                case moveBackstage:
//                    drive.followTrajectorySequenceAsync(pathToBackStage);
//                    aState = Auto_State.stop;
//                    break;
//                case detectAprilTag:
//                    dashboardTelemetry.addLine("align aprilTag updating");
//                    switch (cState) {
//                        case init:
//                            cState = Command_State.execute;
//                            break;
//                        case execute:
//                            double[] idData = myCamera.getAprilTagIDData(9);
//                            drive.alignAprilTag(25.0, 0.0, 0.0, idData[0], idData[1], idData[2], idData[3]);
//                            if (drive.isEndAlign()) {
//                                cState = Command_State.end;
//                            }
//                        case end:
//                            aState = Auto_State.stop;
//                            dashboardTelemetry.addLine("STOP Now");
//                            drive.updateRobotDrive(0.0, 0.0, 0.0, 0.0);
//                            break;
//                    }
//                    break;
//                case stop:
//                    dashboardTelemetry.addLine("STOP Now");
//                    drive.updateRobotDrive(0.0, 0.0, 0.0, 0.0);
//                    break;
//            }



            currentTime = timer.milliseconds();
            dashboardTelemetry.addLine("run-time: " + currentTime/1000);
            dashboardTelemetry.addLine("loop-time: " + (currentTime - lastTime)/1000);

//            if (currentTime > 7000.0 && !isAutoEnd) {
//                drive.followTrajectorySequenceAsync(leftPark);
//                isAutoEnd = true;
//            }

            lastTime = currentTime;
//            dashboardTelemetry.update();
        }
    }
}
