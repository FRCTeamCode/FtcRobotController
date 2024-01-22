package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.hardware.ArmAuto;
import org.firstinspires.ftc.teamcode.hardware.CameraPro;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
@Config
public class AutoDoubleCamera extends LinearOpMode {
    CameraPro cameraPro;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    private Boolean isCameraUpdated = false;
    SampleMecanumFaster drive;
    ArmAuto armAuto;
    Claw claw;
    Intake intake;
    AprilTagDetection tagOfInterest = null;
    String targetSide = "";
    TrajectorySequence targetRoad;
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;
    double alingnTagTime = 0;
    private int tagID = 5;
    private boolean isAutoEnd;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumFaster(hardwareMap, dashboardTelemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(AutoConstants.START);

        armAuto = new ArmAuto(hardwareMap, dashboardTelemetry);
        claw = new Claw(hardwareMap, dashboardTelemetry);
        intake = new Intake(hardwareMap, dashboardTelemetry);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(camera, 20);
//        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId2);
//        FtcDashboard.getInstance().startCameraStream(camera2, 20);

        isAutoEnd = false;
        cameraPro = new CameraPro(hardwareMap, dashboardTelemetry, false);

        TrajectorySequence pathLeft = drive.trajectorySequenceBuilder(AutoConstants.START)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0.0, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutLowPixel);
                })
                .addTemporalMarker(0.9, () -> {
                    claw.lowClaw();
                })
                .addTemporalMarker(1.6, () -> {
                    intake.openIntake();
                })
                .addTemporalMarker(2.0, () -> {
                    claw.middleClaw();
                })
                .lineToLinearHeading(AutoConstants.BL1_PUT)
                .addTemporalMarker(3.0, () -> {
                    intake.closeIntake();
                })
                .addTemporalMarker(3.0, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
                })
                .addTemporalMarker(3.6, () -> {
                    claw.lowerClaw();
                })
                .lineToLinearHeading(AutoConstants.BL1_PUT_tag)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    alingnTagTime = timer.milliseconds();
                })
//                .lineToLinearHeading(AutoConstants.BL1_BACKSTAGE)
//                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
//                    armAuto.setArmPos(AutoConstants.autoPutArm);
//                })
//                .waitSeconds(2.0)
//                .lineToLinearHeading(AutoConstants.BL1_Tag)
//
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    armAuto.setArmPos(1.8);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    claw.middleClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    armAuto.setArmPos(0.58);
//                })
//                .lineToLinearHeading(AutoConstants.BL1_STOP)
//                .lineToLinearHeading(AutoConstants.BL1_STOP_BACK)

                .build();

        TrajectorySequence pathLeftPark = drive.trajectorySequenceBuilder(AutoConstants.BL1_Tag)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
//                .addTemporalMarker(0.0, () -> {
//                    armAuto.setArmPos(1.08);
//                })
//                .addTemporalMarker(0.9, () -> {
//                    claw.lowClaw();
//                })
//                .addTemporalMarker(1.6, () -> {
//                    intake.openIntake();
//                })
//                .addTemporalMarker(2.0, () -> {
//                    claw.middleClaw();
//                })
//                .lineToLinearHeading(AutoConstants.BL1_PUT)
//                .addTemporalMarker(3.0, () -> {
//                    intake.closeIntake();
//                })
//                .addTemporalMarker(3.0, () -> {
//                    armAuto.setArmPos(AutoConstants.autoPutArmPre);
//                })
//                .addTemporalMarker(3.6, () -> {
//                    claw.lowerClaw();
//                })
//                .lineToLinearHeading(AutoConstants.BL1_PUT_tag)
                .lineToLinearHeading(AutoConstants.BL1_BACKSTAGE)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armAuto.setArmPos(AutoConstants.autoPutArm);
                })
                .waitSeconds(2.0)
                .lineToLinearHeading(AutoConstants.BL1_Tag)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(1.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    claw.middleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armAuto.setArmPos(0.58);
                })
                .lineToLinearHeading(AutoConstants.BL1_STOP)
                .lineToLinearHeading(AutoConstants.BL1_STOP_BACK)

                .build();

        //Init Mode
        while (!isStarted() && !isStopRequested()) {
            cameraPro.loop();
            dashboardTelemetry.addLine("init: ");
            dashboardTelemetry.update();
//            targetRoad = pathLeft;
//            tagID = 2;
//            targetSide = "Middle";
            targetRoad = pathLeft;
            tagID = 1;
            targetSide = "Left";
            telemetry.addData("AisEndAl", drive.isEndAlign());
        }
        drive.followTrajectorySequenceAsync(targetRoad);
        //Enable Mode
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
                    drive.alinTag(idData, tagID, 11.0, 0.0, -1.2, timer.milliseconds());
//                        drive.followTrajectorySequenceAsync(pathLeftPark);
                } else if (targetSide == "Middle") {
                    drive.alinTag(idData, tagID, 11.0, 0.0, -0.75, timer.milliseconds());
//                        drive.followTrajectorySequenceAsync(pathMiddlePark);
                } else {
                    drive.alinTag(idData, tagID, 11.0, 0.0, 0.0, timer.milliseconds());
//                        drive.followTrajectorySequenceAsync(pathRightPark);
                }
                dashboardTelemetry.addLine("AlignTagTime: " + (timer.milliseconds()-alingnTagTime));
                telemetry.addData("Auto is End Align2", drive.isEndAlign());
//                dashboardTelemetry.addLine("AprilTag Tracking" + cameraPro.getAprilTagIDData(tagID)[0] + ", " + cameraPro.getAprilTagIDData(tagID)[1] + ", " + cameraPro.getAprilTagIDData(tagID)[2] + ", " + cameraPro.getAprilTagIDData(tagID)[3] + ";" );
                if (drive.isHasFindTarget()&&drive.isEndAlign()&&!isAutoEnd/*&&(timer.milliseconds()-alingnTagTime) > 3700&&(Math.abs(drive.getMyPose().getY()-35.46) < 1.0)&&(Math.abs(drive.getMyPose().getHeading()-277.2) < 3.75)*/) {
                    dashboardTelemetry.addLine("Parked now");
                    isAutoEnd = true;
                    if (targetSide == "Left") {
                        drive.setPoseEstimate(AutoConstants.BL1_Tag);
                        drive.followTrajectorySequenceAsync(pathLeftPark);
                    } else if (targetSide == "Middle") {
//                        drive.followTrajectorySequenceAsync(pathMiddlePark);
                    } else {
//                        drive.followTrajectorySequenceAsync(pathRightPark);
                    }
                }

            }

            dashboardTelemetry.update();
        }

    }
}
