package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelIntake extends SequentialCommandGroup {

    public PixelIntake(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new ArmControl(arm, 1.25),
                new ClawControl(claw, 1.0),
                new WaitCommand(200),
                new IntakeControl(intake, 1.0),
                new ArmControl(arm, 0.6),
                new WaitCommand(200),
                new ArmControl(arm, 0.0)
        );
        addRequirements(arm, claw, intake);
    }
}
