package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelHighIntake extends SequentialCommandGroup {

    public PixelHighIntake(Arm arm, Claw claw, Intake intake, Elevator ele) {
        addCommands(
                new EleControl(ele, 4.0),
                new WaitCommand(100),
                new ArmControl(arm, 1.1),
                new ClawControl(claw, 1.2),
                new WaitCommand(100),
                new IntakeControl(intake, 1.5),
                new ArmControl(arm, 0.96)
        );
        addRequirements(arm, claw, intake);
    }
}
