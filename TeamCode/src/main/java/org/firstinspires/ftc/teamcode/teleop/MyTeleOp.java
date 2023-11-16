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
import org.firstinspires.ftc.teamcode.command.AlignAprilTag;
import org.firstinspires.ftc.teamcode.command.ArmControl;
import org.firstinspires.ftc.teamcode.command.ClawControl;
import org.firstinspires.ftc.teamcode.command.IntakeControl;
import org.firstinspires.ftc.teamcode.command.PixelHold;
import org.firstinspires.ftc.teamcode.command.PixelIntake;
import org.firstinspires.ftc.teamcode.command.PixelPut;
import org.firstinspires.ftc.teamcode.command.TeleopDrive;
import org.firstinspires.ftc.teamcode.drive.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;

@TeleOp
public class MyTeleOp extends CommandOpMode {
    private MyCamera myCamera;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    @Override
    public void initialize() {
        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);
        DrivePose drive = new DrivePose(hardwareMap, dashboardTelemetry);
        drive.setDefaultCommand(new TeleopDrive(drive, gamepad1));

//        TestMotor shooter = new TestMotor(hardwareMap, dashboardTelemetry);
        Arm arm  = new Arm(hardwareMap, dashboardTelemetry);
        Claw claw  = new Claw(hardwareMap, dashboardTelemetry);
        Intake intake  = new Intake(hardwareMap, dashboardTelemetry);
//        schedule(new CameraStream(myCamera, gamepad1));

        Button a = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        a.whenPressed(new PixelIntake(arm, claw, intake));
        Button b = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        b.whenPressed(new PixelHold(arm, claw, intake));
        Button x = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        x.whenPressed(new PixelPut(arm, claw, intake));

        Button dl = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_LEFT);
        dl.whenPressed(new IntakeControl(intake,2.0));
        Button dr = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_RIGHT);
        dr.whenPressed(new IntakeControl(intake,1.0));

//        Button y = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
//        y.whenPressed(new PixelIntake(arm, claw));

//        Button x = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X);
//        x.whenPressed(new MovePosition(shooter,100));
//        Button y = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
//        y.whenPressed(new MovePosition(shooter,800));

        Button lb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER);
        lb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 1, 10.0, 5.0, 0.0));
        Button rb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);
        rb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 5, 7.0, 0.0, 0.0));

    }
}






//        schedule(new RunCommand(dashboardTelemetry::update));
//        FtcDashboard.getInstance().startCameraStream(null, 0);