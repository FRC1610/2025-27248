package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.HashMap;
import java.util.Map;

public class DecodePaths {


    ///  BLUE FAR
    public static final Pose BLUE_FAR_START = new Pose(60, 8.75, Math.toRadians(-90));
    public static final Pose BLUE_FAR_SHOOT = new Pose(60, 18.75, Math.toRadians(-90));
    public static final Pose BLUE_FAR_SHOOT_TO_SPIKE3 = new Pose(47.75, 24, Math.toRadians(180));
    public static final Pose BLUE_FAR_PICKUP_SPIKE3_PART1 = new Pose(33, 24, Math.toRadians(180));
    public static final Pose BLUE_FAR_PICKUP_SPIKE3_PART2 = new Pose(16.5, 24, Math.toRadians(180));
    public static final Pose BLUE_FAR_LEAVE = new Pose(60, 34.5, Math.toRadians(-90));


    /// RED FAR
    public static final Pose RED_FAR_START = new Pose(84, 8.75, Math.toRadians(-90));
    public static final Pose RED_FAR_SHOOT = new Pose(84, 18.75, Math.toRadians(-90));
    public static final Pose RED_FAR_SHOOT_TO_SPIKE3 = new Pose(96.75, 34.5, 0);
    public static final Pose RED_FAR_PICKUP_SPIKE3_PART1 = new Pose(111, 34.5, 0);
    public static final Pose RED_FAR_PICKUP_SPIKE3_PART2 = new Pose(118, 34.5, 0);
    public static final Pose RED_FAR_LEAVE = new Pose(84, 34.5, Math.toRadians(-90));


    /// BLUE CLOSE
    public static final Pose BLUE_NEAR_START = new Pose(23,119.8,Math.toRadians(-90));
    public static final Pose BLUE_NEAR_SHOOT = new Pose(59, 84, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_GOTO_SPIKE1 = new Pose(41.5, 84, Math.toRadians(180));
    public static final Pose BLUE_NEAR_PICKUP_SPIKE1_PART1 = new Pose(33, 84, Math.toRadians(180));
    public static final Pose BLUE_NEAR_PICKUP_SPIKE1_PART2 = new Pose(16.5, 84, Math.toRadians(180));
    public static final Pose BLUE_NEAR_LEAVE = new Pose(44, 71.5, Math.toRadians(-135));

    ///  RED CLOSE
    public static final Pose RED_NEAR_START = new Pose(121,119.8,Math.toRadians(-90));
    public static final Pose RED_NEAR_SHOOT = new Pose(84.5, 84, Math.toRadians(-90));
    public static final Pose RED_NEAR_GOTO_SPIKE1 = new Pose(100, 84, 0);
    public static final Pose RED_NEAR_PICKUP_SPIKE1_PART1 = new Pose(111, 84, 0);
    public static final Pose RED_NEAR_PICKUP_SPIKE1_PART2 = new Pose(118, 84, 0);
    public static final Pose RED_NEAR_LEAVE = new Pose(102, 71.5, Math.toRadians(-45));

    /// BLUE CLOSE LEAVE ONLY
    public static final Pose BLUE_NEAR_FROM_SHOOT_TO_WALL = new Pose(59, 133.6, Math.toRadians(-90));

    // RED CLOSE LEAVE ONLY
    public static final Pose RED_NEAR_FROM_SHOOT_TO_WALL = new Pose(85, 133.6, Math.toRadians(-90));


    /// PATH CHAINING METHODS

    public enum AUTO_PATHS {
        NEAR_PATH_TO_SHOOT_AREA,
        NEAR_SHOOT_AREA_TO_SPIKE1,
        NEAR_PICKUP_SPIKE1_PART1,
        NEAR_PICKUP_SPIKE1_PART2,
        NEAR_GOTO_SHOOT_SPIKE1,
        NEAR_SHOOT_LEAVE,


        FAR_START_TO_SHOOT,
        FAR_SHOOT_TO_SPIKE3_LINEUP,
        FAR_SPIKE3_PICKUP_PART1,
        FAR_SPIKE3_PICKUP_PART2,
        FAR_SPIKE3_TO_SHOOT,
        FAR_SHOOT_LEAVE,

        NEAR_SHOOT_TO_WALL,
    }

    public static PathChain buildPath(RobotHardware robot, Follower follower, Pose blueFrom, Pose blueTo, Pose redFrom, Pose redTo) {
        Pose from = robot.allianceColorBlue ? blueFrom : redFrom;
        Pose to = robot.allianceColorBlue ? blueTo : redTo;
        return PedroConstants.buildPath(follower, from, to);
    }

    public static Map<AUTO_PATHS, PathChain> buildPaths(RobotHardware robot, Follower follower) {
        Map<AUTO_PATHS, PathChain> paths = new HashMap<>();

        /// NEAR
        paths.put(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA, buildPath(robot, follower,
                BLUE_NEAR_START, BLUE_NEAR_SHOOT,
                RED_NEAR_START, RED_NEAR_SHOOT));

        paths.put(AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1, buildPath(robot, follower,
                BLUE_NEAR_SHOOT, BLUE_NEAR_GOTO_SPIKE1,
                RED_NEAR_SHOOT, RED_NEAR_GOTO_SPIKE1));

        paths.put(AUTO_PATHS.NEAR_PICKUP_SPIKE1_PART1, buildPath(robot, follower,
                BLUE_NEAR_GOTO_SPIKE1, BLUE_NEAR_PICKUP_SPIKE1_PART1,
                RED_NEAR_GOTO_SPIKE1, RED_NEAR_PICKUP_SPIKE1_PART1));

        paths.put(AUTO_PATHS.NEAR_PICKUP_SPIKE1_PART2, buildPath(robot, follower,
                BLUE_NEAR_PICKUP_SPIKE1_PART1, BLUE_NEAR_PICKUP_SPIKE1_PART2,
                RED_NEAR_PICKUP_SPIKE1_PART1, RED_NEAR_PICKUP_SPIKE1_PART2));

        paths.put(AUTO_PATHS.NEAR_GOTO_SHOOT_SPIKE1, buildPath(robot, follower,
                BLUE_NEAR_PICKUP_SPIKE1_PART2, BLUE_NEAR_SHOOT,
                RED_NEAR_PICKUP_SPIKE1_PART2, RED_NEAR_SHOOT));

        paths.put(AUTO_PATHS.NEAR_SHOOT_LEAVE, buildPath(robot, follower,
                BLUE_NEAR_SHOOT, BLUE_NEAR_LEAVE,
                RED_NEAR_SHOOT, RED_NEAR_LEAVE));

        /// FAR
        paths.put(AUTO_PATHS.FAR_START_TO_SHOOT, buildPath(robot, follower,
                BLUE_FAR_START, BLUE_FAR_SHOOT,
                RED_FAR_START, RED_FAR_SHOOT));

        paths.put(AUTO_PATHS.FAR_SHOOT_TO_SPIKE3_LINEUP, buildPath(robot, follower,
                BLUE_FAR_SHOOT, BLUE_FAR_SHOOT_TO_SPIKE3,
                RED_FAR_SHOOT, RED_FAR_SHOOT_TO_SPIKE3));

        paths.put(AUTO_PATHS.FAR_SPIKE3_PICKUP_PART1, buildPath(robot, follower,
                BLUE_FAR_SHOOT_TO_SPIKE3, BLUE_FAR_PICKUP_SPIKE3_PART1,
                RED_FAR_SHOOT_TO_SPIKE3, RED_FAR_PICKUP_SPIKE3_PART1));

        paths.put(AUTO_PATHS.FAR_SPIKE3_PICKUP_PART2, buildPath(robot, follower,
                BLUE_FAR_PICKUP_SPIKE3_PART1, BLUE_FAR_PICKUP_SPIKE3_PART2,
                RED_FAR_PICKUP_SPIKE3_PART1, RED_FAR_PICKUP_SPIKE3_PART2));

        paths.put(AUTO_PATHS.FAR_SPIKE3_TO_SHOOT, buildPath(robot, follower,
                BLUE_FAR_PICKUP_SPIKE3_PART2, BLUE_FAR_SHOOT,
                RED_FAR_PICKUP_SPIKE3_PART2, RED_FAR_SHOOT));

        paths.put(AUTO_PATHS.FAR_SHOOT_LEAVE, buildPath(robot, follower,
                BLUE_FAR_SHOOT, BLUE_FAR_LEAVE,
                RED_FAR_SHOOT, RED_FAR_LEAVE));

        /// NEAR LEAVE
        paths.put(AUTO_PATHS.NEAR_SHOOT_TO_WALL, buildPath(robot, follower,
                BLUE_NEAR_SHOOT, BLUE_NEAR_FROM_SHOOT_TO_WALL,
                RED_NEAR_SHOOT, RED_NEAR_FROM_SHOOT_TO_WALL));

        return paths;
    }
}
