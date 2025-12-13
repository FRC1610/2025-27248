package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class DecodePaths {


    ///  BLUE FAR
    // would be useful if we could start turret sideways to get more time
    public static final Pose BLUE_FAR_START = new Pose(60, 8.75, Math.toRadians(-90)); // public static final Pose BlueFarStart = new Pose(57, 9, Math.toRadians(270));
    //public static final Pose BlueFarShoot = new Pose(57,21, Math.toRadians(180));
    public static final Pose BLUE_FAR_TO_SHOOT_AREA = new Pose(60, 80, Math.toRadians(-90));
    public static final Pose BLUE_FAR_TO_SPIKE3 = new Pose(60, 46, Math.toRadians(-90));
    public static final Pose BLUE_FAR_LEAVE = new Pose(60, 40, Math.toRadians(-90));
    //public static final Pose BLUE_FAR_GET_ARTIFACTS = new Pose(24, 36, Math.toRadians(-180));
    //public static final Pose BlueFarSpike = new Pose(25,37, Math.toRadians(180)); // public static final Pose BlueFarSpike = new Pose(25,37, Math.toRadians(180));


    /// RED FAR
    public static final Pose RED_FAR_START = new Pose(84, 8.75, Math.toRadians(-90)); // public static final Pose BlueFarStart = new Pose(57, 9, Math.toRadians(270)); //public static final Pose RedFarStart = new Pose(88,9, Math.toRadians(270));
    public static final Pose RED_FAR_TO_SHOOT_AREA = new Pose(84, 80, Math.toRadians(-90));
    public static final Pose RED_FAR_TO_SPIKE3 = new Pose(84, 35.5, Math.toRadians(-90));
    //public static final Pose RED_FAR_LEAVe = new Pose();
    //public static final Pose RED_FAR_GET_ARTIFACTS = new Pose(120, 36, Math.toRadians(-180));
    //public static final Pose RedFarShoot = new Pose(88,21, Math.toRadians(0));


    // NOTE: i changed the values according to how it appeared in the visualizer

    /// BLUE CLOSE
    public static final Pose BLUE_NEAR_START = new Pose(23,119.8,Math.toRadians(-90)); // public static final Pose BLUE_NEAR_START = new Pose(22.5,120,Math.toRadians(90));
    public static final Pose BLUE_NEAR_SHOOT = new Pose(45, 98, Math.toRadians(-90)); // public static final Pose BLUE_NEAR_SHOOT = new Pose(48, 95, Math.toRadians(90));
    public static final Pose BLUE_NEAR_GOTO_ARTIFACTS = new Pose(24.1, 98, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_PICKUP_ARTIFACTS = new Pose(24.1, 90, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_GO_INSIDE_ZONE = new Pose(60, 108, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_START_TO_LEAVE = new Pose(23, 97, Math.toRadians(-90));

    ///  RED CLOSE
    public static final Pose RED_NEAR_START = new Pose(121,119.8,Math.toRadians(-90)); // public static final Pose BLUE_NEAR_START = new Pose(121,120, Math.toRadians(90));
    public static final Pose RED_NEAR_SHOOT = new Pose(98.5, 98, Math.toRadians(-90)); // public static final Pose BLUE_NEAR_SHOOT = new Pose(96,95, Math.toRadians(90));
    public static final Pose RED_NEAR_GOTO_ARTIFACTS = new Pose(119.8, 98, Math.toRadians(-90));
    public static final Pose RED_NEAR_PICKUP_ARTIFACTS = new Pose(119.8, 90, Math.toRadians(-90));
    public static final Pose RED_NEAR_GO_INSIDE_ZONE = new Pose(84, 108, Math.toRadians(-90));
    public static final Pose RED_NEAR_START_TO_LEAVE = new Pose(121, 97, Math.toRadians(-90));
}
