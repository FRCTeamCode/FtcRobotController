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
    public static final Pose2d BL1_Tag = new Pose2d(20.5, 36.0, Math.toRadians(-81.8));
    public static final Pose2d BL1_STOP = new Pose2d(48.0, 42.0, Math.toRadians(-81.8));

    //Blue near BACKSTAGE side - Middle
    public static final Pose2d BM1_PUT = new Pose2d(20.0, 10.0, 0.0);
    public static final Pose2d BM1_BACKSTAGE = new Pose2d(20.5, 30.0, Math.toRadians(-87.0));
    public static final Pose2d BM1_Tag = new Pose2d(20.5, 40.0, Math.toRadians(-90.0));
    public static final Pose2d BM1_STOP = new Pose2d(15.0, 0.0, 0.0);

    //Blue near BACKSTAGE side - Right
    public static final Pose2d BR1_PUT = new Pose2d(20.0, 10.0, 0.0);
    public static final Pose2d BR1_BACKSTAGE = new Pose2d(20.5, 30.0, Math.toRadians(-87.0));
    public static final Pose2d BR1_Tag = new Pose2d(20.5, 40.0, Math.toRadians(-90.0));
    public static final Pose2d BR1_STOP = new Pose2d(15.0, -10.0, 0.0);


}
