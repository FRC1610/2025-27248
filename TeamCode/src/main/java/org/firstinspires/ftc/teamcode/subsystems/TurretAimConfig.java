package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Panels-configurable turret aim offsets for long-range shots.
 * Values are initialized from {@link Constants} but can be edited live in Panels.
 */
@Configurable
public class TurretAimConfig {
    public static double turretFarAimAdjustBlue = Constants.turret_FAR_AIM_ADJUST_BLUE;
    public static double turretFarAimAdjustRed = Constants.turret_FAR_AIM_ADJUST_RED;

    private TurretAimConfig() {
        // Utility holder; no instances required.
    }
}
