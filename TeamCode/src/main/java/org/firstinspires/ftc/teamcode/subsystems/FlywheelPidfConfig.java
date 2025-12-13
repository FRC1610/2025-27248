package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Panels-configurable PIDF constants for the launcher flywheel.
 * Values are initialized from {@link Constants} but can be edited live in Panels.
 */
@Configurable
public class FlywheelPidfConfig {
    public static double launcherP = Constants.LAUNCHER_P;
    public static double launcherI = Constants.LAUNCHER_I;
    public static double launcherD = Constants.LAUNCHER_D;
    public static double launcherF = Constants.LAUNCHER_F;

    private FlywheelPidfConfig(RobotHardware robot) { // add robot to class
        launcherF = robot.TELEOP_LAUNCHER_F;
        // Utility holder; no instances required.
    }
}
