package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;
import org.firstinspires.ftc.teamcode.subsystem.TestMotor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class CommonAuto extends LinearOpMode {
    SampleMecanumFaster drive;
    MyCamera myCamera;
    Turret turret;
    TestMotor testMotor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    double initTime = 0;
    double currentTime = 0;
    public static double cycleDelay = .52;
    public static double turretAfterScoreDelay = .7;
    private boolean isAutoEnd = false;
    private Pose2d lastPose = new Pose2d(11.96, 0.4, 0.0);

    private enum Auto_State {
        drivePose, alignAprilTag, PARK, STOP
    }
    private enum Command_State {
        init, execute, end, isFinish
    }
    Auto_State aState = Auto_State.drivePose;
    Command_State cState = Command_State.init;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumFaster(hardwareMap, dashboardTelemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(AutoConstants.START);

        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);

//        turret = new Turret();
//        turret.init(hardwareMap, dashboardTelemetry);
//        turret.setTargetAngle(0);

//        testMotor = new TestMotor(hardwareMap, dashboardTelemetry);

        while (!isStarted() && !isStopRequested()) {
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
//            dashboardTelemetry.addData("minT", dashboardTelemetry.getMsTransmissionInterval());
            dashboardTelemetry.addData("Error0", 0.0);
            dashboardTelemetry.addData("Error1", 0.0);
            dashboardTelemetry.addData("Error2", 0.0);
            dashboardTelemetry.addData("Error3", 0.0);
            dashboardTelemetry.update();
            sleep(20);
        }

        while (!isStopRequested() && opModeIsActive()) {
            dashboardTelemetry.addData("timer", timer.milliseconds());
            dashboardTelemetry.addData("AprilTagX", myCamera.getAprilTagIDData(10)[2]);
            dashboardTelemetry.update();
            drive.update();
            switch (aState) {
                case drivePose:
                    dashboardTelemetry.addLine("drive pose updating");
                    switch (cState) {
                        case init:
                            initTime = timer.milliseconds();
                            cState = Command_State.isFinish;
                            break;
                        case execute:
                            drive.autoMoveXY(12.0, 0.3, 0.0, 0.3, timer.milliseconds() - initTime, 8000.0, 10.0, 0.6);
                            cState = Command_State.isFinish;
                            break;
                        case end:
                            aState = Auto_State.alignAprilTag;
                            cState = Command_State.init;
                            break;
                        case isFinish:
                            if (drive.isEndAutoMove()) {
                                drive.initAuto();
                                cState = Command_State.end;
                            } else {
                                cState = Command_State.execute;
                            }
                            break;
                    }
                    break;
                case alignAprilTag:
                    dashboardTelemetry.addLine("align aprilTag updating");
                    switch (cState) {
                        case init:
                            cState = Command_State.isFinish;
                            break;
                        case execute:
                            double[] idData = myCamera.getAprilTagIDData(9);
                            drive.alignAprilTag(25.0, 0.0, 0.0, idData[0], idData[1], idData[2], idData[3]);
                            cState = Command_State.isFinish;
                            break;
                        case end:
                            aState = Auto_State.PARK;
                            cState = Command_State.init;
                            break;
                        case isFinish:
                            if (drive.isEndAlign()) {
                                cState = Command_State.end;
                                drive.initAuto();
                            } else {
                                cState = Command_State.execute;
                            }
                            break;
                    }
                    break;
                case PARK:
                    dashboardTelemetry.addLine("drive park");
                    switch (cState) {
                        case init:
                            initTime = timer.milliseconds();
                            cState = Command_State.isFinish;
                            break;
                        case execute:
                            drive.autoMoveXY(0.0, 0.3, 0.0, 0.3, timer.milliseconds() - initTime, 8000.0, 10.0, 0.6);
                            cState = Command_State.isFinish;
                            break;
                        case end:
                            aState = Auto_State.STOP;
                            cState = Command_State.init;
                            break;
                        case isFinish:
                            if (drive.isEndAutoMove()) {
                                cState = Command_State.end;
                                drive.initAuto();
                            } else {
                                cState = Command_State.execute;
                            }
                            break;
                    }
                    break;
                case STOP:
                    dashboardTelemetry.addLine("STOP Now");
                    drive.updateRobotDrive(0.0, 0.0, 0.0, 0.0);
                    break;
            }
        }
    }
}
