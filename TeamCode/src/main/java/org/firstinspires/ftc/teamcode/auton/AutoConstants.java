package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class AutoConstants {

    // Cycle parking constraints
    public static final TrajectoryVelocityConstraint PARK_VEL = SampleMecanumDrive.getVelocityConstraint(24, 4.0, 10.2);
    public static final TrajectoryAccelerationConstraint PARK_ACCEL = SampleMecanumDrive.getAccelerationConstraint(24);

    // GLOBAL
    public static final Pose2d START = new Pose2d(0.0, 0.0, 0.0);

    //Blue near BACKSTAGE side - Left
    public static final Pose2d BL1_PUT = new Pose2d(20.0, 9.5, 0.0);
    public static final Pose2d BL1_BACKSTAGE = new Pose2d(20.5, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BL1_Tag = new Pose2d(20.5, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BL1_STOP = new Pose2d(48.0, 42.0, Math.toRadians(-81.8));

    //Blue near BACKSTAGE side - Middle
    public static final Pose2d BM1_PUT = new Pose2d(34.0, 13.0, Math.toRadians(-56.0));
    public static final Pose2d BM1_BACKSTAGE = new Pose2d(22.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BM1_Tag = new Pose2d(22.0, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BM1_STOP = new Pose2d(48.0, 42.0, Math.toRadians(-81.8));

    //Blue near BACKSTAGE side - Right
    public static final Pose2d BR1_PUT = new Pose2d(22.0, -6.36, Math.toRadians(-40.0));
    public static final Pose2d BR1_BACKSTAGE = new Pose2d(29.0, 41.0, Math.toRadians(-81.8));
    public static final Pose2d BR1_Tag = new Pose2d(29.0, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BR1_STOP = new Pose2d(48.0, 42.0, Math.toRadians(-81.8));


    //Blue far BACKSTAGE side - Left
    public static final Pose2d BL2_PUT2 = new Pose2d(22.0, 7.5, Math.toRadians(36.0));
    public static final Pose2d BL2_way0 = new Pose2d(4.0, 0.0, Math.toRadians(18.0));
    public static final Pose2d BL2_way1 = new Pose2d(4.0, 8.0, Math.toRadians(-80.5));
    public static final Pose2d BL2_way2 = new Pose2d(4.0, 55.0, Math.toRadians(-80.5));
    public static final Pose2d BL2_BACKSTAGE0 = new Pose2d(17.6, 55.0, Math.toRadians(-80.5));
    public static final Pose2d BL2_BACKSTAGE1 = new Pose2d(17.6, 94.5, Math.toRadians(-80.5));
    public static final Pose2d BL2_Tag2 = new Pose2d(17.6, 85.0, Math.toRadians(-80.5));
    public static final Pose2d BL2_STOP2 = new Pose2d(40.0, 92.0, Math.toRadians(-80.5));

    //Blue far BACKSTAGE side - Middle
    public static final Pose2d BM2_PUT = new Pose2d(34.0, 13.0, Math.toRadians(-56.0));
    public static final Pose2d BM2_BACKSTAGE = new Pose2d(22.0, 40.0, Math.toRadians(-81.8));
    public static final Pose2d BM2_Tag = new Pose2d(22.0, 34.0, Math.toRadians(-81.8));
    public static final Pose2d BM2_STOP = new Pose2d(48.0, 42.0, Math.toRadians(-81.8));

    //Blue far BACKSTAGE side - Right1 - after put pixel, run straight to BACKSTAGE
    public static final Pose2d BR2_PUT = new Pose2d(31.0, 2.7, Math.toRadians(-80.0));
    public static final Pose2d BR2_BACKSTAGE = new Pose2d(22.0, 93.75, Math.toRadians(-80.0));
    public static final Pose2d BR2_Tag = new Pose2d(22.0, 85.0, Math.toRadians(-80.0));
    public static final Pose2d BR2_STOP = new Pose2d(40.0, 92.0, Math.toRadians(-80.0));

    //Blue far BACKSTAGE side - Right2 - after put pixel, run straight to BACKSTAGE
    public static final Pose2d BR2_PUT2 = new Pose2d(21.5, -8.25, Math.toRadians(-2.0));
    public static final Pose2d BR2_way1 = new Pose2d(2.0, 12.0, Math.toRadians(-81.5));
    public static final Pose2d BR2_way2 = new Pose2d(2.0, 52.0, Math.toRadians(-81.5));
    public static final Pose2d BR2_BACKSTAGE2 = new Pose2d(22.5, 94.5, Math.toRadians(-80.5));
    public static final Pose2d BR2_Tag2 = new Pose2d(22.5, 85.0, Math.toRadians(-80.5));
    public static final Pose2d BR2_STOP2 = new Pose2d(40.0, 92.0, Math.toRadians(-80.5));


}
