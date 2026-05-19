package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Field Centric Drive Test", group = "Testing")
public class FieldCentricDriveTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            robot.pinpoint.update();
            robot.updateHeadingOffsetFromAllianceButton();

            double botHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double adjustedHeading = robot.applyHeadingOffset(botHeading);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.FieldCentricDrive(x, y, rx, adjustedHeading);

            telemetry.addData("Heading (rad)", botHeading);
            telemetry.addData("Heading offset (rad)", robot.getHeadingOffsetRadians());
            telemetry.addData("Adjusted heading (rad)", adjustedHeading);
            telemetry.update();
        }
    }
}
