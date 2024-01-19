package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelPutLow extends SequentialCommandGroup {

    public PixelPutLow(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new ClawControl(claw, 2.0),
                new ArmControl(arm, 1.8),
                new ClawControl(claw, 7.0),
//                new WaitCommand(200),
//                new ArmControl(arm, 2.8),
//                new WaitCommand(200),
                new ArmControl(arm, 2.87)
        );
        addRequirements(arm, claw, intake);
    }
}
