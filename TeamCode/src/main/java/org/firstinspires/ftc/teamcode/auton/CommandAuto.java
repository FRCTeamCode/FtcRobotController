package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;

public class CommandAuto  extends CommandOpMode {
    private SampleMecanumFaster drive;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumFaster(hardwareMap, dashboardTelemetry);


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                )
        );
    }

    @Override
    public void run() {

    }
}
