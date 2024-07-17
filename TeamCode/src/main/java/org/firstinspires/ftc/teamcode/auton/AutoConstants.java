package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class AutoConstants {

    public static Boolean isFieldControl = false;
    public static Boolean isBlueOrRed = true;
    public static Boolean isUp = false;
    public static Boolean isDown = false;
    public  static double initAngle = AngleUnit.DEGREES.toRadians(90.0);
    public static double isOpRevise = 1.0;
    public  static  double autoPutArmPre = 1.8;
    public  static  double autoPutArm = 2.2;

    // Cycle parking constraints
    public static final TrajectoryVelocityConstraint PARK_VEL = SampleMecanumDrive.getVelocityConstraint(24, 5.0, 10.2);
    public static final TrajectoryAccelerationConstraint PARK_ACCEL = SampleMecanumDrive.getAccelerationConstraint(24);

    public static final TrajectoryVelocityConstraint PARK_VEL2 = SampleMecanumDrive.getVelocityConstraint(32, 5.0, 10.2);
    public static final TrajectoryAccelerationConstraint PARK_ACCEL2 = SampleMecanumDrive.getAccelerationConstraint(32);

    // GLOBAL
    public static final Pose2d START = new Pose2d(0.0, 0.0, 0.0);

    //Blue near BACKSTAGE side - Left
    public static final Pose2d BL1_PUT = new Pose2d(20.0, 9.25-3.15, 0.0);
    public static final Pose2d BL1_PUT_tag = new Pose2d(23.5, 10, Math.toRadians(-92.0));
    public static final Pose2d BL1_BACKSTAGE = new Pose2d(23.5, 40.0-3.15, Math.toRadians(-92.0));
    public static final Pose2d BL1_Tag = new Pose2d(20.5, 28.0, Math.toRadians(-92.0));
    public static final Pose2d BL1_STOP = new Pose2d(3.0, 28.0, Math.toRadians(-92.0));
    public static final Pose2d BL1_STOP_BACK = new Pose2d(3.0, 42.0, Math.toRadians(-92.0));

    //Red near BACKSTAGE side - Right
    public static final Pose2d RR1_PUT = new Pose2d(19.6, -9.25+3.5, 0.0);
    public static final Pose2d RR1_BACKSTAGE = new Pose2d(24.0, -40.0+3.15, Math.toRadians(92.0));
    public static final Pose2d RR1_Tag = new Pose2d(24.0, -28.0, Math.toRadians(92.0));
    public static final Pose2d RR1_STOP = new Pose2d(3.0, -28.0, Math.toRadians(92.0));
    public static final Pose2d RR1_STOP_BACK = new Pose2d(3.0, -42.0, Math.toRadians(92.0));

    //Blue near BACKSTAGE side - Middle
    public static final Pose2d BM1_PUT = new Pose2d(34.0, 10.0-3.0, Math.toRadians(-56.0));
    public static final Pose2d BM1_BACKSTAGE = new Pose2d(29.3, 40.0-3.15, Math.toRadians(-92.0));
    public static final Pose2d BM1_Tag = new Pose2d(29.3, 28.0, Math.toRadians(-92.0));
    public static final Pose2d BM1_STOP = new Pose2d(3.0, 28.0, Math.toRadians(-92.0));
    public static final Pose2d BM1_STOP_BACK = new Pose2d(3.0, 42.0, Math.toRadians(-92.0));

    //Red near BACKSTAGE side - Middle
    public static final Pose2d RM1_PUT = new Pose2d(34.8, -9.0+3.0, Math.toRadians(58.0));
    public static final Pose2d RM1_BACKSTAGE = new Pose2d(30.5, -40.0+3.15, Math.toRadians(92.0));
    public static final Pose2d RM1_Tag = new Pose2d(30.5, -28.0, Math.toRadians(92.0));
    public static final Pose2d RM1_STOP = new Pose2d(3.0, -28.0, Math.toRadians(92.0));
    public static final Pose2d RM1_STOP_BACK = new Pose2d(3.0, -42.0, Math.toRadians(92.0));

    //Blue near BACKSTAGE side - Right
    public static final Pose2d BR1_PUT = new Pose2d(22.0, -6.5-4.0, Math.toRadians(-40.0));
    public static final Pose2d BR1_BACKSTAGE = new Pose2d(36.0, 41.0-3.15, Math.toRadians(-92.0));
    public static final Pose2d BR1_Tag = new Pose2d(36.0, 28.0, Math.toRadians(-92.0));
    public static final Pose2d BR1_STOP = new Pose2d(3.0, 28.0, Math.toRadians(-92.0));
    public static final Pose2d BR1_STOP_BACK = new Pose2d(3.0, 42.0, Math.toRadians(-92.0));

    //Red near BACKSTAGE side - left
    public static final Pose2d RL1_PUT = new Pose2d(22.0, 7.5+3.0, Math.toRadians(40.0));
    public static final Pose2d RL1_BACKSTAGE = new Pose2d(36.0, -41.0+3.15, Math.toRadians(92.0));
    public static final Pose2d RL1_Tag = new Pose2d(36.0, -28.0, Math.toRadians(92.0));
    public static final Pose2d RL1_STOP = new Pose2d(3.0, -28.0, Math.toRadians(92.0));
    public static final Pose2d RL1_STOP_BACK = new Pose2d(3.0, -42.0, Math.toRadians(92.0));


    //Blue far BACKSTAGE side - Left
    public static final Pose2d BL2_way = new Pose2d(24.0, 0.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_way_m = new Pose2d(24.0, 26.5, Math.toRadians(-92.0));
    public static final Pose2d BL2_PUT = new Pose2d(31.5, 26.5, Math.toRadians(-92.0));
    public static final Pose2d BL2_PUTBack = new Pose2d(31.5, 35.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_way0 = new Pose2d(25.0, 48.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_way1 = new Pose2d(20.0, 60.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_way2 = new Pose2d(25.0, 80.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_BACKSTAGE = new Pose2d(25.0, 97.5, Math.toRadians(-92.0));
    public static final Pose2d BL2_Tag = new Pose2d(25.0, 80.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_BACKSTAGE_back = new Pose2d(25.0, 87.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_STOP = new Pose2d(3.0, 87.0, Math.toRadians(-92.0));
    public static final Pose2d BL2_STOP_BACK = new Pose2d(3.0, 102.0, Math.toRadians(-92.0));
    //Red far BACKSTAGE side - Right
    public static final Pose2d RR2_way = new Pose2d(24.0, 0.0, Math.toRadians(92.0));
    public static final Pose2d RR2_way_m = new Pose2d(24.0, -27.75, Math.toRadians(92.0));
    public static final Pose2d RR2_PUT = new Pose2d(27.0, -27.0, Math.toRadians(92.0));
    public static final Pose2d RR2_PUTBack = new Pose2d(27.0, -29.5, Math.toRadians(92.0));
    public static final Pose2d RR2_way0 = new Pose2d(40.0+6.0, -32.0, Math.toRadians(92.0));
    public static final Pose2d RR2_way1 = new Pose2d(40.0, -76.0, Math.toRadians(92.0));
    public static final Pose2d RR2_way2 = new Pose2d(16.0, -80.0, Math.toRadians(92.0));
    public static final Pose2d RR2_BACKSTAGE = new Pose2d(16.0, -96.25, Math.toRadians(92.0));

    //Blue far BACKSTAGE side - Middle
    public static final Pose2d BM2_PUT = new Pose2d(29.36, 4.0, Math.toRadians(0.0));
    public static final Pose2d BM2_way = new Pose2d(26.5, 24.0, Math.toRadians(-92.0));
    public static final Pose2d BM2_BACKSTAGE_way = new Pose2d(32.5, 80.0, Math.toRadians(-92.0));
    public static final Pose2d BM2_BACKSTAGE = new Pose2d(32.5, 97.0, Math.toRadians(-92.0));
    public static final Pose2d BM2_Tag = new Pose2d(32.5, 80.0, Math.toRadians(-92.0));
    public static final Pose2d BM2_BACKSTAGE_back = new Pose2d(32.5, 87.0, Math.toRadians(-92.0));
    public static final Pose2d BM2_STOP = new Pose2d(5.0, 87.0, Math.toRadians(-92.0));
    public static final Pose2d BM2_STOP_BACK = new Pose2d(5.0, 102.0, Math.toRadians(-92.0));

    //Red far BACKSTAGE side - Middle
    public static final Pose2d RM2_PUT = new Pose2d(29.6, -4.0, Math.toRadians(0.0));
    public static final Pose2d RM2_way = new Pose2d(26.5, -12.0, Math.toRadians(92.0));
    public static final Pose2d RM2_BACKSTAGE_way = new Pose2d(17.5, -80.0, Math.toRadians(92.0));
    public static final Pose2d RM2_BACKSTAGE = new Pose2d(17.5, -95.8, Math.toRadians(92.0));

    //Blue far BACKSTAGE side - Right - after put pixel, run straight to BACKSTAGE
    public static final Pose2d BR2_PUT = new Pose2d(25.0+7.0, 2.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_PUTBack = new Pose2d(25.0+7.0, 5.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_way0 = new Pose2d(6.0, 5.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_way1 = new Pose2d(6.0, 74.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_way2 = new Pose2d(36.0, 74.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_BACKSTAGE = new Pose2d(36.0, 98.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_BACKSTAGE_back = new Pose2d(36.0, 88.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_Tag = new Pose2d(36.0, 80.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_STOP = new Pose2d(3.0, 88.0, Math.toRadians(-92.0));
    public static final Pose2d BR2_STOP_BACK = new Pose2d(3.0, 102.0, Math.toRadians(-92.0));

    //Red far BACKSTAGE side - Left - after put pixel, run straight to BACKSTAGE
    public static final Pose2d RL2_PUT = new Pose2d(25.0+7.0, -5.8, Math.toRadians(92.0));
    public static final Pose2d RL2_PUTBack = new Pose2d(25.0+7.0, -8.0, Math.toRadians(92.0));
    public static final Pose2d RL2_way0 = new Pose2d(50.3, -8.0, Math.toRadians(92.0));
    public static final Pose2d RL2_way1 = new Pose2d(45.0, -75.0, Math.toRadians(92.0));
    public static final Pose2d RL2_way2 = new Pose2d(25.0, -75.0, Math.toRadians(92.0));
    public static final Pose2d RL2_BACKSTAGE = new Pose2d(25.0, -96.5, Math.toRadians(92.0));


}
