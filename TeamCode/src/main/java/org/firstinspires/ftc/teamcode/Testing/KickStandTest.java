package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;

// <3.5 ft - 2,200 rpm
// 5-6ft(?) - 2,350 rpm
// max feet - 2,650 rpm

//@Disabled
@TeleOp(name = "Kickstand Test Mode", group = "TeleOp")
public class KickStandTest extends LinearOpMode {
    final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        boolean kickerStandToggled = false;

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) {
                robot.setKickStandPosition(Math.max(0, robot.kickStand1.getPosition() - 0.1));
            }

            if (gamepad1.dpadLeftWasPressed()) {
                robot.setKickStandPosition(Math.min(1, robot.kickStand1.getPosition() + 0.1));
            }

            telemetry.addData("KICKSTAND POSITION", robot.kickStand1.getPosition());

            if (gamepad1.aWasPressed()) {
                if (kickerStandToggled) {
                    robot.setKickStandPosition(Constants.KICKERSTAND_NORMAL);
                } else {
                    robot.setKickStandPosition(Constants.KICKERSTAND_RETRACTED);
                }
                kickerStandToggled = !kickerStandToggled;
            }

            if (gamepad1.bWasPressed()) {
                if (robot.kickStand1.getPosition() == Constants.KICKERSTAND_RETRACTED) {
                    robot.setKickStandPosition(Constants.KICKERSTAND_NORMAL);
                } else {
                    robot.setKickStandPosition(Constants.KICKERSTAND_RETRACTED);
                }
                kickerStandToggled = !kickerStandToggled;
            }

            if (kickerStandToggled) robot.setColorOfBackLights(rgbIndicator.LEDColors.INDIGO);

            telemetry.addLine("RUNNING");
        }
    }

}
