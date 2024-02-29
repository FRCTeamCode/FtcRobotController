package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelPutHighest extends SequentialCommandGroup {

    public PixelPutHighest(Arm arm, Claw claw, Intake intake, Elevator ele) {
        addCommands(
                new ArmControl(arm, 1.8),
                new EleControl(ele, 2.0),
                new ClawControl(claw, 2.0),
                new ArmControl(arm, 2.43),
                new WaitCommand(400),
                new ClawControl(claw, 9.0)
        );
        addRequirements(arm, claw, intake);
    }
}
