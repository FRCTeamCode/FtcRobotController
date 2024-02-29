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
    public static boolean isUp = false;
    public  static double initAngle = AngleUnit.DEGREES.toRadians(90.0);
    public static double isOpRevise = 1.0;
    public  static  double autoPutLowPixel = 0.78;//0.68 -> 0.74
    public  static  double autoPutArmPre = 1.85;
    public  static  double autoPutArm = 2.35;

    // Cycle parking constraints
    public static final TrajectoryVelocityConstraint PARK_VEL = SampleMecanumDrive.getVelocityConstraint(24, 4.0, 10.2);
    public static final TrajectoryAccelerationConstraint PARK_ACCEL = SampleMecanumDrive.getAccelerationConstraint(24);

    // GLOBAL
    public static final Pose2d START = new Pose2d(0.0, 0.0, 0.0);

    //Blue near BACKSTAGE side - Left
    public static final Pose2d BL1_PUT = new Pose2d(21.0, 5.9, 0.0);
    public static final Pose2d BL1_PUT_Back = new Pose2d(14.0, 5.9, 0.0);
    public static final Pose2d BL1_BACKSTAGE = new Pose2d(20.7, 27.0, Math.toRadians(-84.0));
    public static final Pose2d BL1_BACKSTAGE_better = new Pose2d(20.7, 38.0, Math.toRadians(-84.0));
    public static final Pose2d BL1_Tag = new Pose2d(20.7, 30.0, Math.toRadians(-84.0));
    public static final Pose2d BL1_STOP = new Pose2d(3.0, 30.0, Math.toRadians(-84.0));
    public static final Pose2d BL1_STOP_BACK = new Pose2d(3.0, 40.0, Math.toRadians(-84.0));

    //Red near BACKSTAGE side - Right
    public static final Pose2d RR1_PUT = new Pose2d(21.0, -5.9, 0.0);
    public static final Pose2d RR1_PUT_Back = new Pose2d(14.0, -5.9, 0.0);
    public static final Pose2d RR1_BACKSTAGE = new Pose2d(20.3, -27.0, Math.toRadians(84.0));
    public static final Pose2d RR1_BACKSTAGE_better = new Pose2d(20.3, -38.0, Math.toRadians(84.0));
    public static final Pose2d RR1_Tag = new Pose2d(20.3, -30.0, Math.toRadians(84.0));
    public static final Pose2d RR1_STOP = new Pose2d(3.0, -30.0, Math.toRadians(84.0));
    public static final Pose2d RR1_STOP_BACK = new Pose2d(3.0, -40.0, Math.toRadians(84.0));

    //Blue near BACKSTAGE side - Middle
    public static final Pose2d BM1_PUT = new Pose2d(35.5, 9.3, Math.toRadians(-56.0));
    public static final Pose2d BM1_BACKSTAGE = new Pose2d(25.0, 38.7, Math.toRadians(-84.0));
    public static final Pose2d BM1_Tag = new Pose2d(25.0, 30.0, Math.toRadians(-84.0));
    public static final Pose2d BM1_STOP = new Pose2d(3.0, 30.0, Math.toRadians(-84.0));
    public static final Pose2d BM1_STOP_BACK = new Pose2d(3.0, 40.0, Math.toRadians(-84.0));

    //Red near BACKSTAGE side - Middle
    public static final Pose2d RM1_PUT = new Pose2d(35.5, -9.3, Math.toRadians(56.0));
    public static final Pose2d RM1_BACKSTAGE = new Pose2d(25.0, -38.7, Math.toRadians(84.0));
    public static final Pose2d RM1_Tag = new Pose2d(25.0, -30.0, Math.toRadians(84.0));
    public static final Pose2d RM1_STOP = new Pose2d(3.0, -30.0, Math.toRadians(84.0));
    public static final Pose2d RM1_STOP_BACK = new Pose2d(3.0, -40.0, Math.toRadians(84.0));

    //Blue near BACKSTAGE side - Right
    public static final Pose2d BR1_PUT = new Pose2d(21.7, -11.7, Math.toRadians(-40.0));
    public static final Pose2d BR1_BACKSTAGE = new Pose2d(32.15, 38.2, Math.toRadians(-84.0));
    public static final Pose2d BR1_Tag = new Pose2d(32.15, 30.0, Math.toRadians(-84.0));
    public static final Pose2d BR1_STOP = new Pose2d(3.0, 30.0, Math.toRadians(-84.0));
    public static final Pose2d BR1_STOP_BACK = new Pose2d(3.0, 40.0, Math.toRadians(-84.0));
    //Red near BACKSTAGE side - left
    public static final Pose2d RL1_PUT = new Pose2d(21.7, 11.7, Math.toRadians(40.0));
    public static final Pose2d RL1_BACKSTAGE = new Pose2d(32.15, -38.2, Math.toRadians(84.0));
    public static final Pose2d RL1_Tag = new Pose2d(32.15, -30.0, Math.toRadians(84.0));
    public static final Pose2d RL1_STOP = new Pose2d(3.0, -30.0, Math.toRadians(84.0));
    public static final Pose2d RL1_STOP_BACK = new Pose2d(3.0, -40.0, Math.toRadians(84.0));


    //Blue far BACKSTAGE side - Left
    public static final Pose2d BL2_way = new Pose2d(24.0, 0.0, Math.toRadians(-84.0));
    public static final Pose2d BL2_way_m = new Pose2d(24.0, 28.4, Math.toRadians(-84.0));
    public static final Pose2d BL2_PUT = new Pose2d(27.5, 28.4, Math.toRadians(-84.0));
    public static final Pose2d BL2_PUTBack = new Pose2d(27.5, 31.8, Math.toRadians(-84.0));
    public static final Pose2d BL2_way0 = new Pose2d(46.0, 31.8, Math.toRadians(-84.0));
    public static final Pose2d BL2_way1 = new Pose2d(46.0, 80.0, Math.toRadians(-84.0));
    public static final Pose2d BL2_way2 = new Pose2d(16.0, 80.0, Math.toRadians(-84.0));
    public static final Pose2d BL2_BACKSTAGE = new Pose2d(10.0, 99.5, Math.toRadians(-84.0));
    public static final Pose2d BL2_Tag = new Pose2d(10.0, 90.0, Math.toRadians(-84.0));
    //Red far BACKSTAGE side - Right
    public static final Pose2d RR2_way = new Pose2d(24.0, 0.0, Math.toRadians(84.0));
    public static final Pose2d RR2_way_m = new Pose2d(24.0, -28.4, Math.toRadians(84.0));
    public static final Pose2d RR2_PUT = new Pose2d(27.5, -28.4, Math.toRadians(84.0));
    public static final Pose2d RR2_PUTBack = new Pose2d(27.5, -31.8, Math.toRadians(84.0));
    public static final Pose2d RR2_way0 = new Pose2d(50.0, -31.8, Math.toRadians(84.0));
    public static final Pose2d RR2_way1 = new Pose2d(44.0, -80.0, Math.toRadians(84.0));
    public static final Pose2d RR2_way2 = new Pose2d(13.0, -78.0, Math.toRadians(84.0));
    public static final Pose2d RR2_BACKSTAGE = new Pose2d(10.0, -99.0, Math.toRadians(84.0));
    public static final Pose2d RR2_Tag = new Pose2d(10.0, -90.0, Math.toRadians(84.0));

    //Blue far BACKSTAGE side - Middle
    public static final Pose2d BM2_PUT = new Pose2d(30.0, 4.0, Math.toRadians(0.0));
    public static final Pose2d BM2_way = new Pose2d(26.5, 24.0, Math.toRadians(-84.0));
    public static final Pose2d BM2_way0 = new Pose2d(23.0, 32.0, Math.toRadians(-84.0));
    public static final Pose2d BM2_BACKSTAGE_way = new Pose2d(23.0, 80.0, Math.toRadians(-84.0));
    public static final Pose2d BM2_BACKSTAGE = new Pose2d(23.0, 99.0, Math.toRadians(-84.0));
    public static final Pose2d BM2_Tag = new Pose2d(23.0, 90.0, Math.toRadians(-84.0));
    //Red far BACKSTAGE side - Middle
    public static final Pose2d RM2_PUT = new Pose2d(30.0, -4.0, Math.toRadians(0.0));
    public static final Pose2d RM2_way = new Pose2d(26.5, -24.0, Math.toRadians(84.0));
    public static final Pose2d RM2_way0 = new Pose2d(23.0, -24.0, Math.toRadians(84.0));
    public static final Pose2d RM2_BACKSTAGE_way = new Pose2d(23.0, -80.0, Math.toRadians(84.0));
    public static final Pose2d RM2_BACKSTAGE = new Pose2d(23.0, -99.0, Math.toRadians(84.0));
    public static final Pose2d RM2_Tag = new Pose2d(23.5, -90.0, Math.toRadians(84.0));

    //Blue far BACKSTAGE side - Right - after put pixel, run straight to BACKSTAGE
    public static final Pose2d BR2_PUT = new Pose2d(25.0+5.8, 6.8, Math.toRadians(-84.0));
    public static final Pose2d BR2_PUTBack = new Pose2d(25.0+5.8, 8.0, Math.toRadians(-84.0));
    public static final Pose2d BR2_way0 = new Pose2d(45.0+5.3, 8.0, Math.toRadians(-84.0));
    public static final Pose2d BR2_way1 = new Pose2d(40.0, 80.0, Math.toRadians(-84.0));
    public static final Pose2d BR2_way2 = new Pose2d(23.5, 78.0, Math.toRadians(-84.0));
    public static final Pose2d BR2_BACKSTAGE = new Pose2d(23.5, 99.0, Math.toRadians(-84.0));
    public static final Pose2d BR2_Tag = new Pose2d(23.5, 90.0, Math.toRadians(-84.0));

    //Red far BACKSTAGE side - Left - after put pixel, run straight to BACKSTAGE
    public static final Pose2d RL2_PUT = new Pose2d(30.8, -6.4, Math.toRadians(84.0));
    public static final Pose2d RL2_PUTBack = new Pose2d(30.8, -8.0, Math.toRadians(84.0));
    public static final Pose2d RL2_way0 = new Pose2d(53.0, -8.0, Math.toRadians(84.0));
    public static final Pose2d RL2_way1 = new Pose2d(44.0, -80.0, Math.toRadians(84.0));
    public static final Pose2d RL2_way2 = new Pose2d(23.5, -78.0, Math.toRadians(84.0));
    public static final Pose2d RL2_BACKSTAGE = new Pose2d(23.5, -99.0, Math.toRadians(84.0));
    public static final Pose2d RL2_Tag = new Pose2d(23.5, -90.0, Math.toRadians(84.0));


}
