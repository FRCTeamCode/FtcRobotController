package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelPut extends SequentialCommandGroup {

    public PixelPut(Arm arm, Claw claw, Intake intake) {
        addCommands(new ParallelCommandGroup(
                        new ClawControl(claw, 3.0),
                        new IntakeControl(intake, 3.0)
                ),
                new ArmControl(arm, 1200)
        );
        addRequirements(arm, claw);
    }
}
