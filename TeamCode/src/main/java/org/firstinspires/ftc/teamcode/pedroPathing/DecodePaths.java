package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class DecodePaths {


    ///  BLUE FAR
    public static final Pose BlueFarStart = new Pose(57, 9, Math.toRadians(270));
    public static final Pose BlueFarShoot = new Pose(57,21, Math.toRadians(180));
    public static final Pose BlueFarSpike = new Pose(25,37, Math.toRadians(180));


    /// RED FAR
    public static final Pose RedFarStart = new Pose(88,9, Math.toRadians(270));
    public static final Pose RedFarShoot = new Pose(88,21, Math.toRadians(0));


    /// BLUE CLOSE, ORIGINAL X 22.5, Y 120, HEADING 90 and X 48, Y 95, HEADING 90
    /// NOTE: i changed the values according to how it appeared in the visualizer
    public static final Pose BLUE_NEAR_START = new Pose(23,119.8,Math.toRadians(-90));
    public static final Pose BLUE_NEAR_SHOOT = new Pose(45, 98, Math.toRadians(-90));
    public static final Pose BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_1AND3 = new Pose(24.1, 98, Math.toRadians(-90));
    public static final Pose BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_2 = new Pose(24.1, 91, Math.toRadians(-90));

    ///  RED CLOSE

    public static final Pose RedNearStart = new Pose (121,120, Math.toRadians(90));
    public static final Pose RedNearShoot = new Pose (96,95, Math.toRadians(90));
}
