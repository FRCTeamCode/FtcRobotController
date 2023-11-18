package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelPut extends SequentialCommandGroup {

    public PixelPut(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new ClawControl(claw, 2.0),
                new ArmControl(arm, 400),
                new WaitCommand(200),
                new ClawControl(claw, 3.0),
                new ArmControl(arm, 850),
                new WaitCommand(300),
                new ArmControl(arm, 915)
        );
        addRequirements(arm, claw, intake);
    }
}
