package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class PixelArmResetCom extends CommandBase {
    private Arm arm;
    private Claw claw;
    private Intake intake;
    private Elevator ele;
    private double mElePos;
    private boolean isBackNow, isArmSafe;
    ElapsedTime timer = new ElapsedTime();

    public PixelArmResetCom(Arm arm, Claw claw, Intake intake, Elevator ele) {
        this.arm = arm;
        this.claw = claw;
        this.intake = intake;
        this.ele = ele;
        addRequirements(this.arm, this.claw, this.intake,this.ele);
    }

    @Override
    public void initialize() {
        mElePos = ele.getPosition();
        isArmSafe = false;
        isBackNow = false;
        ele.defaultEle();
        arm.setArmPos(1.35);
        timer.reset();
    }

    @Override
    public void execute() {
        if (arm.getPotVol() < 2.0) {
            if (!isArmSafe) {
                isArmSafe = true;
                claw.middleClaw();
                intake.closeIntake();
            }
            if (mElePos > 0.5) {
                if (timer.milliseconds() > 3000) {
                    isBackNow = true;
                }
            } else {
//                if (timer.milliseconds() > 1.5) {
//                    isBackNow = true;
//                }
                if (timer.milliseconds() > (mElePos - 0.185) * 4800.0) {
                    isBackNow = true;
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmPos(0.56);
    }

    @Override
    public boolean isFinished() {
        return isBackNow;
    }
}