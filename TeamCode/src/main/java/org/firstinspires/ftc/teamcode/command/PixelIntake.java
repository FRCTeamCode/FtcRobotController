package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelIntake extends SequentialCommandGroup {

    public PixelIntake(Arm arm, Claw claw, Intake intake) {
        addCommands(new ParallelCommandGroup(
                        new ClawControl(claw, 1.0),
                        new IntakeControl(intake, 1.0)
                ),
                new ArmControl(arm, 300)
        );
        addRequirements(arm, claw);
    }
}
