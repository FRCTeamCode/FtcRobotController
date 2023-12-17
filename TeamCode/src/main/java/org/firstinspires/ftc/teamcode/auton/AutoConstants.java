package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class AutoConstants {

    public static Boolean isFieldControl = true;
    public static Boolean isBlueOrRed = true;
    public  static double initAngle = AngleUnit.DEGREES.toRadians(90.0);
    public static double isOpRevise = 1.0;

    // Cycle parking constraints
    public static final TrajectoryVelocityConstraint PARK_VEL = SampleMecanumDrive.getVelocityConstraint(24, 4.0, 10.2);
    public static final TrajectoryAccelerationConstraint PARK_ACCEL = SampleMecanumDrive.getAccelerationConstraint(24);

    // GLOBAL
    public static final Pose2d START = new Pose2d(0.0, 0.0, 0.0);
    private static final double offsetCamera = 3.15;

    //Blue near BACKSTAGE side - Left
    public static final Pose2d BL1_PUT = new Pose2d(20.0, 9.25, 0.0);
    public static final Pose2d BL1_BACKSTAGE = new Pose2d(20.5, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BL1_Tag = new Pose2d(20.5, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BL1_STOP = new Pose2d(46.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BL1_STOP_BACK = new Pose2d(46.0, 47.0, Math.toRadians(-81.8));

    //Red near BACKSTAGE side - Right
    public static final Pose2d RR1_PUT = new Pose2d(20.0, -9.5, 0.0);
    public static final Pose2d RR1_BACKSTAGE = new Pose2d(20.5, -40.0, Math.toRadians(81.8));
    public static final Pose2d RR1_Tag = new Pose2d(20.5, -34.0, Math.toRadians(81.8));
    public static final Pose2d RR1_STOP = new Pose2d(46.0, -40.0, Math.toRadians(81.8));
    public static final Pose2d RR1_STOP_BACK = new Pose2d(46.0, -47.0, Math.toRadians(81.8));

    //Blue near BACKSTAGE side - Middle
    public static final Pose2d BM1_PUT = new Pose2d(35.0, 12.8, Math.toRadians(-56.0));
    public static final Pose2d BM1_BACKSTAGE = new Pose2d(22.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BM1_Tag = new Pose2d(22.0, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BM1_STOP = new Pose2d(46.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BM1_STOP_BACK = new Pose2d(46.0, 47.0, Math.toRadians(-81.8));

    //Red near BACKSTAGE side - Middle
    public static final Pose2d RM1_PUT = new Pose2d(34.0, -13.0, Math.toRadians(56.0));
    public static final Pose2d RM1_BACKSTAGE = new Pose2d(22.0, -40.0, Math.toRadians(81.8));
    public static final Pose2d RM1_Tag = new Pose2d(22.0, -34.0, Math.toRadians(81.8));
    public static final Pose2d RM1_STOP = new Pose2d(46.0, -40.0, Math.toRadians(81.8));
    public static final Pose2d RM1_STOP_BACK = new Pose2d(46.0, -47.0, Math.toRadians(81.8));

    //Blue near BACKSTAGE side - Right
    public static final Pose2d BR1_PUT = new Pose2d(22.0, -6.5, Math.toRadians(-40.0));
    public static final Pose2d BR1_BACKSTAGE = new Pose2d(29.0, 41.0, Math.toRadians(-81.8));
    public static final Pose2d BR1_Tag = new Pose2d(29.0, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BR1_STOP = new Pose2d(46.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BR1_STOP_BACK = new Pose2d(46.0, 47.0, Math.toRadians(-81.8));
    //Red near BACKSTAGE side - left
    public static final Pose2d RL1_PUT = new Pose2d(22.0, 6.36, Math.toRadians(40.0));
    public static final Pose2d RL1_BACKSTAGE = new Pose2d(29.0, -41.0, Math.toRadians(81.8));
    public static final Pose2d RL1_Tag = new Pose2d(29.0, -34.0, Math.toRadians(81.8));
    public static final Pose2d RL1_STOP = new Pose2d(46.0, -40.0, Math.toRadians(81.8));
    public static final Pose2d RL1_STOP_BACK = new Pose2d(46.0, -47.0, Math.toRadians(81.8));


    //Blue far BACKSTAGE side - Left
    public static final Pose2d BL2_way = new Pose2d(24.5+5.0, 0.0-2.8, Math.toRadians(-82.0));
    public static final Pose2d BL2_PUT2 = new Pose2d(24.5+5.0, 27.75-2.8, Math.toRadians(-82.0));
    public static final Pose2d BL2_PUT2Back = new Pose2d(24.5+5.0, 29.5-2.8, Math.toRadians(-82.0));
    public static final Pose2d BL2_way0 = new Pose2d(40.0+5.0, 29.5-2.8, Math.toRadians(-82.0));
    public static final Pose2d BL2_way1 = new Pose2d(40.0+5.0, 80.0-2.8, Math.toRadians(-82.0));
    public static final Pose2d BL2_way2 = new Pose2d(7.0+4.0, 80.0-2.8, Math.toRadians(-82.0));
    public static final Pose2d BL2_BACKSTAGE = new Pose2d(7.0+4.0, 96.5-2.8, Math.toRadians(-82.0));

    //Blue far BACKSTAGE side - Middle
    public static final Pose2d BM2_PUT = new Pose2d(34.0, 13.0, Math.toRadians(-56.0));
    public static final Pose2d BM2_BACKSTAGE = new Pose2d(22.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BM2_Tag = new Pose2d(22.0, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BM2_STOP = new Pose2d(48.0, 42.0, Math.toRadians(-81.8));

    //Blue far BACKSTAGE side - Right - after put pixel, run straight to BACKSTAGE
    public static final Pose2d BR2_PUT2 = new Pose2d(25.0+5.5, 6.0-3.2, Math.toRadians(-81.0));
    public static final Pose2d BR2_PUT2Back = new Pose2d(25.0+5.5, 8.0-3.2, Math.toRadians(-81.0));
    public static final Pose2d BR2_way0 = new Pose2d(45.0+5.3, 8.0-3.15, Math.toRadians(-81.0));
    public static final Pose2d BR2_way1 = new Pose2d(35.0+5.3, 80.0-3.15, Math.toRadians(-81.0));
    public static final Pose2d BR2_way2 = new Pose2d(22.75, 80.0-3.15, Math.toRadians(-81.0));
    public static final Pose2d BR2_BACKSTAGE = new Pose2d(22.75, 96.5-3.15, Math.toRadians(-81.0));


}
