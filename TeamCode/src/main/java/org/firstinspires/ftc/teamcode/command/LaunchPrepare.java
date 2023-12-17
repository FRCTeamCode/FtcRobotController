package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class LaunchPrepare extends SequentialCommandGroup {
    public LaunchPrepare(Arm arm, Claw claw, Intake intake) {
        addCommands(
                new ClawControl(claw, 2.0),
                new ArmControl(arm, 2.0),
                new ClawControl(claw, 4.0),
                new WaitCommand(300),
                new ArmControl(arm, 3.1),
                new ReviseDirec2()
        );
        addRequirements(arm, claw, intake);
    }
}
