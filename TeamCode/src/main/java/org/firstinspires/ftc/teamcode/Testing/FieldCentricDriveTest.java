package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Field-centric drive test OpMode that uses Limelight pose data to re-seed the odometry position
 * when available. The current alliance selection flips the field frame so drivers keep the same
 * joystick orientation from either side of the field.
 */
@TeleOp(name = "Field Centric Drive Test", group = "Test")
public class FieldCentricDriveTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            robot.refreshLimelightResult();
            LLResult result = robot.getLatestLimelightResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                Position position = botpose.getPosition();
                Orientation orientation = botpose.getOrientation();
                Pose2D pose2D = new Pose2D(
                        DistanceUnit.METER,
                        position.getX(DistanceUnit.METER),
                        position.getY(DistanceUnit.METER),
                        orientation.getYaw(AngleUnit.RADIANS));
                robot.pinpoint.setPosition(pose2D);
                telemetry.addLine("Pose updated from Limelight");
            }

            robot.pinpoint.update();

            double heading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            boolean allianceRed = robot.refreshAllianceFromSwitchState();
            double fieldHeading = heading + (allianceRed ? Math.PI : 0.0);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            robot.FieldCentricDrive(x, y, rotation, fieldHeading);

            Pose2D pose = robot.pinpoint.getPosition();
            telemetry.addData("Alliance", allianceRed ? "RED" : "BLUE");
            telemetry.addData("Heading (rad)", heading);
            telemetry.addData("Field Heading (rad)", fieldHeading);
            telemetry.addData("Pose", pose);
            telemetry.update();

            sleep(10);
        }
    }
}
