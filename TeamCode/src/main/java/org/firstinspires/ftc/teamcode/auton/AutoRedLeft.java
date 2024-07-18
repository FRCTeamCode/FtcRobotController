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
import org.firstinspires.ftc.teamcode.subsystem.CameraAuto;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class AutoRedLeft extends LinearOpMode {
    SampleMecanumFaster drive;
    CameraPro cameraPro;
    ArmAuto armAuto;
    Claw claw;
    Intake intake;
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
        drive.setPoseEstimate(AutoConstants.START);

        armAuto = new ArmAuto(hardwareMap, dashboardTelemetry);
        claw = new Claw(hardwareMap, dashboardTelemetry);
        intake = new Intake(hardwareMap, dashboardTelemetry);
        cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, true);

        isAutoEnd = false;

        TrajectorySequence pathRight = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)
                .lineToLinearHeading(AutoConstants.RR2_way)
                .lineToLinearHeading(AutoConstants.RR2_way_m)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(1.08);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowClaw();
                })
                .lineToLinearHeading(AutoConstants.RR2_PUT)
//                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(AutoConstants.RR2_PUTBack)
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.middleClaw();
                })
//                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeIntake();
                })
//                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(0.58);
                })
//                .waitSeconds(1.5)
                .lineToLinearHeading(AutoConstants.RR2_way0)
                .lineToLinearHeading(AutoConstants.RR2_way1)
                .lineToLinearHeading(AutoConstants.RR2_way2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowerClaw();
                })
                .build();
        TrajectorySequence pathRightPark = drive.trajectorySequenceBuilder(AutoConstants.RR2_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)

                .lineToLinearHeading(AutoConstants.RR2_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(2.1)
                .lineToLinearHeading(AutoConstants.RR2_BACKSTAGE_back)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.RR2_STOP)
                .lineToLinearHeading(AutoConstants.RR2_STOP_BACK)
                .build();

        TrajectorySequence pathMiddle = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(1.08);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    claw.lowClaw();
                })
                .lineToLinearHeading(AutoConstants.RM2_PUT)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.middleClaw();
                })
//                .waitSeconds(1.0)
                .lineToLinearHeading(AutoConstants.RM2_way)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armAuto.setArmPos(0.58);
                })
                .waitSeconds(0.1)//wait another robot
                .lineToLinearHeading(AutoConstants.RM2_BACKSTAGE_way)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowerClaw();
                })
                .build();
        TrajectorySequence pathMiddlePark = drive.trajectorySequenceBuilder(AutoConstants.RM2_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)

                .lineToLinearHeading(AutoConstants.RM2_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(2.1)
                .lineToLinearHeading(AutoConstants.RM2_BACKSTAGE_back)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.RM2_STOP)
                .lineToLinearHeading(AutoConstants.RM2_STOP_BACK)
                .build();

        TrajectorySequence pathLeft = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL2)
                .setAccelConstraint(AutoConstants.PARK_ACCEL2)
                .lineToLinearHeading(AutoConstants.RL2_PUT)
                .addTemporalMarker(0.2, () -> {
                    armAuto.setArmPos(1.08);
                })
                .addTemporalMarker(1.0, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.7, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(AutoConstants.RL2_PUTBack)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.middleClaw();
                })
//                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeIntake();
                })
//                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(0.58);
                })
                .waitSeconds(0.1)//wait another robot
                .lineToLinearHeading(AutoConstants.RL2_way0)
                .lineToLinearHeading(AutoConstants.RL2_way1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .lineToLinearHeading(AutoConstants.RL2_way2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.lowerClaw();
                })
                .build();
        TrajectorySequence pathLeftPark = drive.trajectorySequenceBuilder(AutoConstants.RL2_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)

                .lineToLinearHeading(AutoConstants.RL2_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(2.1)
                .lineToLinearHeading(AutoConstants.RL2_BACKSTAGE_back)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.0);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.RL2_STOP)
                .lineToLinearHeading(AutoConstants.RL2_STOP_BACK)
                .build();

        targetRoad = pathRight;

        while (!isStarted() && !isStopRequested()) {
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
                    if (recognition.getLabel() == "RedCube") {
                        tagFound = true;
                        if (Math.abs(x-288) < 48 && Math.abs(y-156) < 60 && Math.abs(recognition.getWidth()-82) < 24 && Math.abs(recognition.getHeight()-97) < 24) {
                            tagID = 4;
                            targetSide = "Left";
                            targetRoad = pathLeft;
                        } else if (Math.abs(x-510) < 48 && Math.abs(y-135) < 60 && Math.abs(recognition.getWidth()-87) < 24 && Math.abs(recognition.getHeight()-82) < 24) {
                            tagID = 5;
                            targetSide = "Middle";
                            targetRoad = pathMiddle;
                        } else {
                            tagID = 6;
                            targetSide = "Right";
                            targetRoad = pathRight;
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
                    dashboardTelemetry.addLine("Don't see tag of interest :(");

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
                cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, false);
                isCameraUpdated = true;
                dashboardTelemetry.addLine("Camera updated!");
                drive.initTagPara();
            }
            if (!drive.isBusy()) {
                //path has finished
                double[] idData = cameraPro.getAprilTagIDData(tagID);
                if (targetSide == "Left") {
                    drive.alinTag(idData, tagID, 17.7, -0.3, -0.25, timer.milliseconds());
                } else if (targetSide == "Middle") {
                    drive.alinTag(idData, tagID, 17.7, 0.0, -0.75, timer.milliseconds());
                } else {
                    drive.alinTag(idData, tagID, 17.7, 0.0, -0.0, timer.milliseconds());
                }
                if (drive.isHasFindTarget()&&drive.isEndAlign()&&!isAutoEnd) {
                    dashboardTelemetry.addLine("Parked now");
                    isAutoEnd = true;
                    if (targetSide == "Left") {
                        drive.setPoseEstimate(AutoConstants.RL2_Tag);
                        drive.followTrajectorySequenceAsync(pathLeftPark);
                    } else if (targetSide == "Middle") {
                        drive.setPoseEstimate(AutoConstants.RM2_Tag);
                        drive.followTrajectorySequenceAsync(pathMiddlePark);
                    } else {
                        drive.setPoseEstimate(AutoConstants.RR2_Tag);
                        drive.followTrajectorySequenceAsync(pathRightPark);
                    }
                }

            }
//            dashboardTelemetry.update();
        }

    }
}
