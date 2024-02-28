package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelPutMiddle extends SequentialCommandGroup {

    public PixelPutMiddle(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new ClawControl(claw, 2.0),
                new ArmControl(arm, 1.8),
//                new WaitCommand(200),
//                new ArmControl(arm, 2.3),
//                new WaitCommand(400),
                new ArmControl(arm, 2.74),
                new WaitCommand(400),
                new ClawControl(claw, 8.0)
        );
        addRequirements(arm, claw, intake);
    }
}
