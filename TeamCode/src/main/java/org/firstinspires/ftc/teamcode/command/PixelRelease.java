package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelRelease extends SequentialCommandGroup {
    public PixelRelease(Intake intake) {
        addCommands(
                new IntakeControl(intake, 2.0),
                new WaitCommand(300),
                new IntakeControl(intake, 4.0),
                new WaitCommand(310),
                new IntakeControl(intake, 5.0),
                new WaitCommand(290),
                new IntakeControl(intake, 6.0)
        );
        addRequirements(intake);
    }
}
