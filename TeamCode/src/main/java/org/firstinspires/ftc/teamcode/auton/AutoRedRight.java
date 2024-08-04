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
public class AutoRedRight extends LinearOpMode {
    NavxMicro navxMicro;
    SampleMecanumFaster drive;
    //    MyCamera myCamera;
    CameraAuto cameraAuto;
    ArmAuto armAuto;
    Claw claw;
    Intake intake;
    Elevator elevator;
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
        drive.setPoseEstimate(AutoConstants.redSTART);

        armAuto = new ArmAuto(hardwareMap, dashboardTelemetry);
        claw = new Claw(hardwareMap, dashboardTelemetry);
        intake = new Intake(hardwareMap, dashboardTelemetry);
        elevator = new Elevator(hardwareMap, dashboardTelemetry);
//        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);
        cameraAuto = new CameraAuto(hardwareMap, dashboardTelemetry);
        navxMicro = new NavxMicro(hardwareMap, dashboardTelemetry);

        isAutoEnd = false;

        TrajectorySequence pathRight = drive.trajectorySequenceBuilder(AutoConstants.redSTART)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .lineToLinearHeading(AutoConstants.RR1_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(1.2, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(AutoConstants.RR1_PUT_Back)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeOpenIntake();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.closeIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.RR1_BACKSTAGE)
                .lineToLinearHeading(AutoConstants.RR1_BACKSTAGE_better)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.3)
                .lineToLinearHeading(AutoConstants.RR1_Tag)
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
                .lineToLinearHeading(AutoConstants.RR1_STOP)
                .lineToLinearHeading(AutoConstants.RR1_STOP_BACK)
                .build();

        TrajectorySequence pathMiddle = drive.trajectorySequenceBuilder(AutoConstants.redSTART)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .lineToLinearHeading(AutoConstants.RM1_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(1.6, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.9, () -> {
                    intake.openIntake();
                })
                .addTemporalMarker(3.6, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.6, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .addTemporalMarker(4.2, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.RM1_BACKSTAGE)
//                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
//                    armAuto.setArmPos(2.8);
//                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.3)
                .lineToLinearHeading(AutoConstants.RM1_Tag)
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
                .lineToLinearHeading(AutoConstants.RM1_STOP)
                .lineToLinearHeading(AutoConstants.RM1_STOP_BACK)
                .build();
        TrajectorySequence pathLeft = drive.trajectorySequenceBuilder(AutoConstants.redSTART)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .lineToLinearHeading(AutoConstants.RL1_PUT)
                .addTemporalMarker(0.1, () -> {
                    elevator.eleIntake();
                })
                .addTemporalMarker(0.5, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(0.9, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(2.1, () -> {
                    intake.openIntake();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(3.8, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.8, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .addTemporalMarker(4.3, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.RL1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(1.3)
                .lineToLinearHeading(AutoConstants.RL1_Tag)
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
                .lineToLinearHeading(AutoConstants.RL1_STOP)
                .lineToLinearHeading(AutoConstants.RL1_STOP_BACK)
                .build();

        targetRoad = pathLeft;

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
                    if (recognition.getLabel() == "RedCube") {
                        tagFound = true;
                        if (Math.abs(x-497) < 48 && Math.abs(y-215) < 48 && Math.abs(recognition.getWidth()-101) < 24 &&
                                Math.abs(recognition.getHeight()-101) < 24) {
                            tagID = 6;
                            targetSide = "Right";
                            targetRoad = pathRight;
                        } else if (Math.abs(x-260) < 48 && Math.abs(y-184) < 48 && Math.abs(recognition.getWidth()-75) < 24 && Math.abs(recognition.getHeight()-85) < 24) {
                            tagID = 5;
                            targetSide = "Middle";
                            targetRoad = pathMiddle;
                        }
                    } else {
                        tagID = 4;
                        targetSide = "Left";
                        targetRoad = pathLeft;
                    }
                }

                if (tagFound) {
                    dashboardTelemetry.addLine("Tag of interest is in sight!\n\nLocation data: " + targetSide);
                } else {
                    tagID = 4;
                    targetSide = "Left";
                    targetRoad = pathLeft;
                    dashboardTelemetry.addLine("Don't see tag of interest :(" + targetSide);

                    if (tagOfInterest == null) {
                        dashboardTelemetry.addLine("(The tag has never been seen1)");
                    } else {
                        dashboardTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:" + targetSide);
                    }
                }

            } else {
                tagID = 4;
                targetSide = "Left";
                targetRoad = pathLeft;
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

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            armAuto.loop();
//            dashboardTelemetry.update();
        }

    }
}
