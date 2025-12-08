package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

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

    private FlywheelPidfConfig() {
        // Utility holder; no instances required.
    }
}
