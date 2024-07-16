package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelArmReset extends SequentialCommandGroup {

    public PixelArmReset(Arm arm, Claw claw, Intake intake, Elevator ele) {
        addCommands(
                new PixelRelease(intake),
//                new WaitCommand(300),
                new PixelArmResetCom(arm, claw, intake, ele)


//                new ArmControl(arm, 1.4),
//                new EleControl(ele, 0.0),
//                new WaitCommand(200),
//                new ClawControl(claw, 2.0),
//                new WaitCommand(300),
//                new IntakeControl(intake, 3.0)
//                new WaitCommand(100),
//                new ArmControl(arm, 1.05),
//                new EleIsBack(ele),
//                new WaitCommand(100),
//                new ArmControl(arm, 0.56)
        );
        addRequirements(arm, claw, intake);
    }
}
