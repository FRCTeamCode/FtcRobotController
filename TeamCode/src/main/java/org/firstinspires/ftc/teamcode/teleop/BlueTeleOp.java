package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.command.AlignAprilTag;
import org.firstinspires.ftc.teamcode.command.ArmControl;
import org.firstinspires.ftc.teamcode.command.CameraStream;
import org.firstinspires.ftc.teamcode.command.ClawControl;
import org.firstinspires.ftc.teamcode.command.DriveAuto;
import org.firstinspires.ftc.teamcode.command.DriveControlType;
import org.firstinspires.ftc.teamcode.command.LaunchFinished;
import org.firstinspires.ftc.teamcode.command.PixelPutLow;
import org.firstinspires.ftc.teamcode.command.PixelPutMiddle;
import org.firstinspires.ftc.teamcode.command.ReverseUp;
import org.firstinspires.ftc.teamcode.command.RevieseDirec;
import org.firstinspires.ftc.teamcode.command.IntakeControl;
import org.firstinspires.ftc.teamcode.command.LaunchPlane;
import org.firstinspires.ftc.teamcode.command.LaunchPrepare;
import org.firstinspires.ftc.teamcode.command.MovePosition;
import org.firstinspires.ftc.teamcode.command.PixelArmReset;
import org.firstinspires.ftc.teamcode.command.PixelHold;
import org.firstinspires.ftc.teamcode.command.PixelIntake;
import org.firstinspires.ftc.teamcode.command.PixelPut;
import org.firstinspires.ftc.teamcode.command.PixelRelease;
import org.firstinspires.ftc.teamcode.command.BlueTeleopDrive;
import org.firstinspires.ftc.teamcode.drive.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Climb;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Launch;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;

@TeleOp
public class BlueTeleOp extends CommandOpMode {
    private MyCamera myCamera;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    @Override
    public void initialize() {
        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);
        AutoConstants.initAngle  = AngleUnit.DEGREES.toRadians(90.0);
        AutoConstants.isBlueOrRed = true;
        DrivePose drive = new DrivePose(hardwareMap, dashboardTelemetry);
        drive.setDefaultCommand(new BlueTeleopDrive(drive, gamepad1));

//        TestMotor shooter = new TestMotor(hardwareMap, dashboardTelemetry);
        Arm arm  = new Arm(hardwareMap, dashboardTelemetry);
        Claw claw  = new Claw(hardwareMap, dashboardTelemetry);
        Intake intake  = new Intake(hardwareMap, dashboardTelemetry);
        Launch launch  = new Launch(hardwareMap, dashboardTelemetry);
        Climb climb = new Climb(hardwareMap, dashboardTelemetry);

        Button lsb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_STICK_BUTTON);
        lsb.whenPressed(new DriveControlType(true, drive));
        Button rsb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_STICK_BUTTON);
        rsb.whenPressed(new DriveControlType(false, drive));

        Button lb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER);
        lb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 2, 6.0, 0.0, 2.9));
        Button rb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);
        rb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 0, 6.0, 0.0, 0.0));

        Button dd = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        dd.whenPressed(new MovePosition(climb,-100));
        Button du = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        du.whenPressed(new MovePosition(climb,-2950));

        Button dl = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_LEFT);
        dl.whenPressed(new RevieseDirec(-1.0));
        Button du0 = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP);
        du0.whenPressed(new ReverseUp(true));
        Button dr = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_RIGHT);
        dr.whenPressed(new RevieseDirec(1.0));


        Button a2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A);
        a2.whenPressed(new PixelHold(arm, claw, intake));
        Button b2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.B);
        b2.whenPressed(new PixelIntake(arm, claw, intake));
        Button x2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        x2.whenPressed(new LaunchFinished(arm, claw, intake));
        Button y2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y);
        y2.whenPressed(new PixelArmReset(arm, claw, intake));

        Button lb2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.LEFT_BUMPER);
        lb2.whenPressed(new IntakeControl(intake,3.0));
        Button rb2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);
//        rb2.whenPressed(new IntakeControl(intake,1.0));
        rb2.whenPressed(new PixelRelease(intake));
        Button s2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        s2.whenPressed(new LaunchPlane(launch, 2.0));
        Button ba2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.BACK);
        ba2.whenPressed(new LaunchPrepare(arm, claw, intake));
//        Button dd2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_DOWN);
//        dd2.whenPressed(new LaunchFinished(arm, claw, intake));
        Button du2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_UP);
        du2.whenPressed(new PixelPut(arm, claw, intake));
        Button dl2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_LEFT);
        dl2.whenPressed(new PixelPutMiddle(arm, claw, intake));
        Button dd2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_DOWN);
        dd2.whenPressed(new PixelPutLow(arm, claw, intake));

//
//        Button a = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A);
//        a.whenPressed(new ArmControl(arm, 2.0));
//        a.whenPressed(new ClawControl(claw, 0.0));
//        a.whenPressed(new DriveAuto(drive, gamepad1));
//        Button b = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B);
//        b.whenPressed(new ArmControl(arm, 1.0));
//        b.whenPressed(new ClawControl(claw, 2.0));
//        Button x = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X);
//        x.whenPressed(new PixelPut(arm, claw, intake));
//        Button y = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
//        y.whenPressed(new PixelArmReset(arm, claw, intake));
//
//        Button dl = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_LEFT);
//        dl.whenPressed(new IntakeControl(intake,2.0));
//        Button dr = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_RIGHT);
//        dr.whenPressed(new IntakeControl(intake,1.0));
//
//        Button lb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER);
//        lb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 1, 10.0, 5.0, 0.0));
//        Button rb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);
//        rb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 5, 7.0, 0.0, 0.0));
//
//        Button a2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A);
//        a2.whenPressed(new IntakeControl(intake, 1));
//        Button b2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.B);
//        b2.whenPressed(new IntakeControl(intake, 2));
//        Button x2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.X);
//        x2.whenPressed(new IntakeControl(intake, 3));
//
//        Button dd2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_DOWN);
//        dd2.whenPressed(new ClawControl(claw, 1));
//        Button dr2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_RIGHT);
//        dr2.whenPressed(new ClawControl(claw, 2));
//        Button du2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_UP);
//        du2.whenPressed(new ClawControl(claw, 3));
//
//        Button lb2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.LEFT_BUMPER);
//        lb2.whenPressed(new ArmControl(arm, 6));
//        Button rb2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);
//        rb2.whenPressed(new ArmControl(arm, 60));
//        Button y2 = new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y);
//        y2.whenPressed(new ArmControl(arm, 890));

    }
}






//        schedule(new RunCommand(dashboardTelemetry::update));
//        FtcDashboard.getInstance().startCameraStream(null, 0);