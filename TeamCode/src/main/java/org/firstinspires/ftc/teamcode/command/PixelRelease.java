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
                new IntakeControl(intake, 2.0),//2.0---0.25
                new WaitCommand(150),
                new IntakeControl(intake, 4.0),//4.0---0.28
                new WaitCommand(150),
                new IntakeControl(intake, 5.0),//5.0---0.3
                new WaitCommand(150),
                new IntakeControl(intake, 6.0)//6.0---0.32
        );
        addRequirements(intake);
    }
}
