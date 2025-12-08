package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class DecodePaths {


    ///  BLUE FAR
    public final Pose BlueFarStart = new Pose(57, 9, Math.toRadians(270));
    public final Pose BlueFarShoot = new Pose(57,21, Math.toRadians(180));
    public final Pose BlueFarSpike = new Pose(25,37, Math.toRadians(180));


    /// RED FAR
    public final Pose RedFarStart = new Pose(88,9, Math.toRadians(270));
    public final Pose RedFarShoot = new Pose(88,21, Math.toRadians(0));


    /// BLUE CLOSE
    public final Pose BlueNearStart = new Pose(22.5,120,Math.toRadians(90));
    public final Pose BlueNearShoot = new Pose(48, 95, Math.toRadians(90));

    ///  RED CLOSE

    public final Pose RedNearStart = new Pose (121,120, Math.toRadians(90));
    public final Pose RedNearShoot = new Pose (96,95, Math.toRadians(90));
}
