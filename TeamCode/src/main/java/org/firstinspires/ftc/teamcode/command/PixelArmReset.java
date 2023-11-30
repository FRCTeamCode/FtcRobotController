package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelArmReset extends SequentialCommandGroup {

    public PixelArmReset(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new IntakeControl(intake, 3.0),
                new ArmControl(arm, 1.2),
                new ClawControl(claw, 2.0),
                new WaitCommand(100),
                new ArmControl(arm, 1.05),
                new WaitCommand(100),
                new ArmControl(arm, 0.57)
        );
        addRequirements(arm, claw, intake);
    }
}
