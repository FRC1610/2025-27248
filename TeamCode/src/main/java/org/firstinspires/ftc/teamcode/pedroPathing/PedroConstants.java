package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {
    public static final double FORWARD_POD_Y = -3.375; // change back to 0 if breaking
    public static final double STRAFE_POD_X = 5.5625;

    public static final FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.5)
            .forwardZeroPowerAcceleration(-27.29365632260416)
            .lateralZeroPowerAcceleration(-65.0582796811887)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06,0,0.0001,0.028))
            .headingPIDFCoefficients(new PIDFCoefficients(0.48,0,0.004,0.0001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06,0,0.0001,0,0.028))
            .centripetalScaling(0.005);

    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(72.30054527943528)
            .yVelocity(52.802258589136315);

    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.2, 1.1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static final PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(FORWARD_POD_Y)
            .strafePodX(STRAFE_POD_X)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathChain buildPath(Follower follower, Pose pose1, Pose pose2) {
        return follower.pathBuilder(pathConstraints)
                .addPath(new BezierLine(pose1, pose2))
                .setConstantHeadingInterpolation(pose2.getHeading())
                .build();
    }
}