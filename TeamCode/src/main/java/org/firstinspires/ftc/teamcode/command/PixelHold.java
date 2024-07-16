package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelHold extends SequentialCommandGroup {

    public PixelHold(Arm arm, Claw claw, Intake intake, Elevator ele) {
        addCommands(
                new IntakeControl(intake,3.0),
                new WaitCommand(400),
                new ParallelCommandGroup(
                        new ClawControl(claw, 2.0),
                        new IntakeControl(intake, 3.0),
                        new EleControl(ele, 0.0)
                ),
                new WaitCommand(200),
                new ArmControl(arm, 0.6)
        );
        addRequirements(arm, claw, intake);
    }
}
