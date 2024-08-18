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
import org.firstinspires.ftc.teamcode.hardware.ArmAuto;
import org.firstinspires.ftc.teamcode.hardware.CameraPro;
import org.firstinspires.ftc.teamcode.hardware.NavxMicro;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class AutoBlueR2ProP extends LinearOpMode {
    NavxMicro navxMicro;
    SampleMecanumFaster drive;
    CameraPro cameraPro;
    ArmAuto armAuto;
    Claw claw;
    Intake intake;
    Elevator elevator;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    AprilTagDetection tagOfInterest = null;
    private Boolean isCameraUpdated = false;
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
        drive.setPoseEstimate(AutoConstants.redSTART);

        armAuto = new ArmAuto(hardwareMap, dashboardTelemetry);
        claw = new Claw(hardwareMap, dashboardTelemetry);
        intake = new Intake(hardwareMap, dashboardTelemetry);
        elevator = new Elevator(hardwareMap, dashboardTelemetry);
        cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, true, false);
        navxMicro = new NavxMicro(hardwareMap, dashboardTelemetry);

        isAutoEnd = false;

        TrajectorySequence pathLeft = drive.trajectorySequenceBuilder(AutoConstants.blueSTART)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)
                .lineToLinearHeading(AutoConstants.BL2pp_way)
                .lineToLinearHeading(AutoConstants.BL2pp_way_m)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    claw.lowClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .lineToLinearHeading(AutoConstants.BL2pp_PUT)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(AutoConstants.BL2pp_PUTBack)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeOpenIntake();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.closeIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BL2pp_way0)
                .waitSeconds(4.5)//wait other robot
                .lineToLinearHeading(AutoConstants.BL2pp_way1)
                .lineToLinearHeading(AutoConstants.BL2pp_way2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowerClaw();
                })
                .build();
        TrajectorySequence pathLeftPark = drive.trajectorySequenceBuilder(AutoConstants.BL2pp_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)

                .lineToLinearHeading(AutoConstants.BL2pp_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.3)
                .lineToLinearHeading(AutoConstants.BL2pp_BACKSTAGE_back)
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
                .lineToLinearHeading(AutoConstants.BL2pp_STOP)
                .build();

        TrajectorySequence pathMiddle = drive.trajectorySequenceBuilder(AutoConstants.blueSTART)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    claw.lowClaw();
                })
                .lineToLinearHeading(AutoConstants.BM2pp_PUT)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(AutoConstants.BM2pp_way)
                .lineToLinearHeading(AutoConstants.BM2pp_way0)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armAuto.setArmPos(0.58);
                })
                .waitSeconds(7.5)//wait
                .lineToLinearHeading(AutoConstants.BM2pp_BACKSTAGE_way)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowerClaw();
                })
                .build();
        TrajectorySequence pathMiddlePark = drive.trajectorySequenceBuilder(AutoConstants.BM2pp_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)

                .lineToLinearHeading(AutoConstants.BM2pp_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.3)
                .lineToLinearHeading(AutoConstants.BM2pp_BACKSTAGE_back)
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
                .lineToLinearHeading(AutoConstants.BM2pp_STOP)
                .build();

        TrajectorySequence pathRight = drive.trajectorySequenceBuilder(AutoConstants.blueSTART)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)
                .lineToLinearHeading(AutoConstants.BR2pp_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(1.3, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.8, () -> {
                    intake.openIntake();
                })
                .addTemporalMarker(3.3, () -> {
                    claw.middleClaw();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(AutoConstants.BR2pp_PUTBack)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeOpenIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    armAuto.setArmPos(0.58);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    intake.closeIntake();
                })
                .lineToLinearHeading(AutoConstants.BR2pp_way0)
                .waitSeconds(3.0)//wait other robot
                .lineToLinearHeading(AutoConstants.BR2pp_way1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .lineToLinearHeading(AutoConstants.BR2pp_way2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowerClaw();
                })
                .build();
        TrajectorySequence pathRightPark = drive.trajectorySequenceBuilder(AutoConstants.BR2pp_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)

                .lineToLinearHeading(AutoConstants.BR2pp_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.3)
                .lineToLinearHeading(AutoConstants.BR2pp_BACKSTAGE_back)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                    elevator.defaultEle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BR2pp_STOP)
                .build();

        targetRoad = pathLeft;

        while (!isStarted() && !isStopRequested()) {
//            cameraPro.loop();
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
            dashboardTelemetry.addData("minT", dashboardTelemetry.getMsTransmissionInterval());

            currentTime = timer.milliseconds();
            dashboardTelemetry.addLine("run-time: " + currentTime/1000);

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
                        if (Math.abs(x-483) < 60 && Math.abs(y-214) < 60 && Math.abs(recognition.getWidth()-98) < 36 && Math.abs(recognition.getHeight()-95) < 36) {
                            tagID = 3;
                            targetSide = "Right";
                            targetRoad = pathRight;
                        } else if (Math.abs(x-248) < 60 && Math.abs(y-187) < 60 && Math.abs(recognition.getWidth()-76) < 36 && Math.abs(recognition.getHeight()-83) < 36) {
                            tagID = 2;
                            targetSide = "Middle";
                            targetRoad = pathMiddle;
                        }
                    } else {
                        tagID = 1;
                        targetSide = "Left";
                        targetRoad = pathLeft;
                    }
                }

                if (tagFound) {
                    dashboardTelemetry.addLine("Tag of interest is in sight!\n\nLocation data: " + targetSide);
                } else {
                    tagID = 1;
                    targetSide = "Left";
                    targetRoad = pathLeft;
                    dashboardTelemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        dashboardTelemetry.addLine("(The tag has never been seen1)" + targetSide);
                    } else {
                        dashboardTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                    }
                }

            } else {
                tagID = 1;
                targetSide = "Left";
                targetRoad = pathLeft;
                dashboardTelemetry.addLine("Don't see tag of interest :(" + targetSide);
                if (tagOfInterest == null) {
                    dashboardTelemetry.addLine("(The tag has never been seen2)" + targetSide);
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
                cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, false, true);
                isCameraUpdated = true;
                dashboardTelemetry.addLine("Camera updated!");
                drive.initTagPara();
            }
            if (!drive.isBusy()) {
                //path has finished
                double[] idData = cameraPro.getAprilTagIDData(tagID);
                if (targetSide == "Left") {
                    drive.alinTag(idData, tagID, 11.0, -0.0, -0.0, timer.milliseconds());
                } else if (targetSide == "Middle") {
                    drive.alinTag(idData, tagID, 11.0, 0.0, -0.75, timer.milliseconds());
                } else {
                    drive.alinTag(idData, tagID, 11.0, 0.4, -0.25, timer.milliseconds());//up-mid,up-left,low-mid
                }
                if (drive.isHasFindTarget()&&drive.isEndAlign()&&!isAutoEnd) {
                    dashboardTelemetry.addLine("Parked now");
                    isAutoEnd = true;
                    if (targetSide == "Left") {
                        drive.setPoseEstimate(AutoConstants.BL2pp_Tag);
                        drive.followTrajectorySequenceAsync(pathLeftPark);
                    } else if (targetSide == "Middle") {
                        drive.setPoseEstimate(AutoConstants.BM2pp_Tag);
                        drive.followTrajectorySequenceAsync(pathMiddlePark);
                    } else {
                        drive.setPoseEstimate(AutoConstants.BR2pp_Tag);
                        drive.followTrajectorySequenceAsync(pathRightPark);
                    }
                }

            }
            dashboardTelemetry.update();
        }

    }
}

