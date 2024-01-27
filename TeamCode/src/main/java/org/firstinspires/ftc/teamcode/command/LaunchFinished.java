package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class LaunchFinished extends SequentialCommandGroup {
    public LaunchFinished(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new ArmControl(arm, 1.3),
                new WaitCommand(100),
                new ClawControl(claw, 2.0),
                new WaitCommand(100),
                new IntakeControl(intake, 3.0),
                new ArmControl(arm, 1.0),
                new WaitCommand(100),
                new ArmControl(arm, 0.56)
        );
        addRequirements(arm, claw, intake);
    }
}
