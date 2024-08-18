package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.hardware.ArmAuto;
import org.firstinspires.ftc.teamcode.hardware.CameraPro;
import org.firstinspires.ftc.teamcode.hardware.NavxMicro;
import org.firstinspires.ftc.teamcode.subsystem.CameraAuto;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class AutoBlueWhitePixel extends LinearOpMode {
    NavxMicro navxMicro;
    SampleMecanumFaster drive;
    //    MyCamera myCamera;
    CameraPro cameraPro;
    ArmAuto armAuto;
    Claw claw;
    Intake intake;
    Elevator elevator;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    AprilTagDetection tagOfInterest = null;
    String targetSide = "";
    private Boolean isCameraUpdated = false;
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
        drive.setPoseEstimate(AutoConstants.redSTART);

        armAuto = new ArmAuto(hardwareMap, dashboardTelemetry);
        claw = new Claw(hardwareMap, dashboardTelemetry);
        intake = new Intake(hardwareMap, dashboardTelemetry);
        elevator = new Elevator(hardwareMap, dashboardTelemetry);
        cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, true, false);
        navxMicro = new NavxMicro(hardwareMap, dashboardTelemetry);

        isAutoEnd = false;

        TrajectorySequence pathLeft = drive.trajectorySequenceBuilder(AutoConstants.blueSTART)
                .setVelConstraint(AutoConstants.PARK_VEL3)
                .setAccelConstraint(AutoConstants.PARK_ACCEL3)
                .lineToLinearHeading(AutoConstants.BL1_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(1.0, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.4)
                .lineToLinearHeading(AutoConstants.BL1_PUT_Back)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeOpenIntake();
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.closeIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BL1_BACKSTAGE)
                .lineToLinearHeading(AutoConstants.BL1_BACKSTAGE_better)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.0)
                .lineToLinearHeading(AutoConstants.BL1_Tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                    elevator.defaultEle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BLWP_STOP)
//                .lineToLinearHeading(AutoConstants.BL1_STOP_BACK)
                .build();
        TrajectorySequence pathLeftPark = drive.trajectorySequenceBuilder(AutoConstants.BLWP_WALL)
                .setVelConstraint(AutoConstants.PARK_VEL3)
                .setAccelConstraint(AutoConstants.PARK_ACCEL3)

                .lineToLinearHeading(AutoConstants.BLWP_BACKSTAGE)
//
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    armAuto.setArmPos(AutoConstants.autoPutArm);
//                })
//                .waitSeconds(1.3)
//                .lineToLinearHeading(AutoConstants.BR2p_BACKSTAGE_back)
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    armAuto.setArmPos(1.8);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    claw.middleClaw();
//                    elevator.defaultEle();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    armAuto.setArmPos(1.0);
//                })
//                .waitSeconds(0.35)
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    armAuto.setArmPos(0.58);
//                })
//                .lineToLinearHeading(AutoConstants.BR2p_STOP)
                .build();

        TrajectorySequence pathMiddle = drive.trajectorySequenceBuilder(AutoConstants.blueSTART)
                .setVelConstraint(AutoConstants.PARK_VEL3)
                .setAccelConstraint(AutoConstants.PARK_ACCEL3)
                .lineToLinearHeading(AutoConstants.BM1_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(1.4, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.8, () -> {
                    intake.openIntake();
                })
                .addTemporalMarker(3.5, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .addTemporalMarker(4.0, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BM1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.0)
                .lineToLinearHeading(AutoConstants.BM1_Tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                    elevator.defaultEle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BM1_STOP)
                .lineToLinearHeading(AutoConstants.BM1_STOP_BACK)
                .build();

        TrajectorySequence pathRight = drive.trajectorySequenceBuilder(AutoConstants.blueSTART)
                .setVelConstraint(AutoConstants.PARK_VEL3)
                .setAccelConstraint(AutoConstants.PARK_ACCEL3)
                .lineToLinearHeading(AutoConstants.BR1_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(1.0, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.0, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(3.6, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.6, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .addTemporalMarker(4.0, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BR1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.0)
                .lineToLinearHeading(AutoConstants.BR1_Tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                    elevator.defaultEle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BR1_STOP)
                .lineToLinearHeading(AutoConstants.BR1_STOP_BACK)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    AutoConstants.initAngle = AngleUnit.DEGREES.toRadians(drive.getImuRad());
                })
                .build();

        targetRoad = pathRight;

        while (!isStarted() && !isStopRequested()) {
            cameraPro.loop();
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
            dashboardTelemetry.addData("minT", dashboardTelemetry.getMsTransmissionInterval());

            currentTime = timer.milliseconds();
            dashboardTelemetry.addLine("run-time: " + currentTime/1000);
            telemetry.addData("navX-Micro init", AutoConstants.isInitNavxMicro2);

            List<Recognition> currentRecognitions = cameraPro.currentRecognitions();
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
                        if (Math.abs(x-260) < 48 && Math.abs(y-217) < 48 && Math.abs(recognition.getWidth()-89) < 24 && Math.abs(recognition.getHeight()-94) < 24) {
                            tagID = 4;
                            targetSide = "Left";
                            targetRoad = pathLeft;
                        } else if (Math.abs(x-480) < 48 && Math.abs(y-187) < 48 && Math.abs(recognition.getWidth()-82) < 24 && Math.abs(recognition.getHeight()-82) < 24) {
                            tagID = 5;
                            targetSide = "Middle";
                            targetRoad = pathMiddle;
                        }
                    } else {
                        tagID = 6;
                        targetSide = "Right";
                        targetRoad = pathRight;
                    }
                }

                if (tagFound) {
                    dashboardTelemetry.addLine("Tag of interest is in sight!\n\nLocation data: " + targetSide);
                } else {
                    tagID = 6;
                    targetSide = "Right";
                    targetRoad = pathRight;
                    dashboardTelemetry.addLine("Don't see tag of interest :(" + targetSide);

                    if (tagOfInterest == null) {
                        dashboardTelemetry.addLine("(The tag has never been seen1)");
                    } else {
                        dashboardTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                    }
                }

            } else {
                tagID = 6;
                targetSide = "Right";
                targetRoad = pathRight;
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
            armAuto.loop();
            cameraPro.loop();

            if (!(!isStarted() && !isStopRequested())&&!isCameraUpdated) {
                cameraPro.visionClose();
                cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, true, true);
                isCameraUpdated = true;
                dashboardTelemetry.addLine("Camera updated!");
                drive.initTagPara();
            }
            if (!drive.isBusy()) {
                //path has finished
                int tagIDWall = 9;
                double[] idData = cameraPro.getAprilTagIDData(tagIDWall);
                if (targetSide == "Left") {
                    drive.alinTagWall(idData, tagIDWall, 15.0, 2.8, 0.0, timer.milliseconds());
                } else if (targetSide == "Middle") {
                    drive.alinTagWall(idData, tagIDWall, 15.0, 2.8, 0.0, timer.milliseconds());
                } else {
                    drive.alinTagWall(idData, tagIDWall, 15.0, 2.8, 0.0, timer.milliseconds());
                }
                if (drive.isHasFindTarget()&&drive.isEndAlign()&&!isAutoEnd) {
                    dashboardTelemetry.addLine("Parked now");
                    isAutoEnd = true;
                    if (targetSide == "Left") {
                        AutoConstants.BLWP_WALL = drive.getMyPose();
                        drive.setPoseEstimate(AutoConstants.BLWP_WALL);
                        dashboardTelemetry.addLine("BLWP_WALL" + AutoConstants.BLWP_WALL);
                        drive.followTrajectorySequenceAsync(pathLeftPark);
                    } else if (targetSide == "Middle") {
                        drive.setPoseEstimate(AutoConstants.BM2p_Tag);
//                        drive.followTrajectorySequenceAsync(pathMiddlePark);
                    } else {
                        drive.setPoseEstimate(AutoConstants.BR2p_Tag);
//                        drive.followTrajectorySequenceAsync(pathRightPark);
                    }
                }

            }
            dashboardTelemetry.update();
        }

    }
}
