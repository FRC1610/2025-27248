package org.firstinspires.ftc.teamcode;

public class Constants {

    public static final double headlightPower = 1;

    ///INTAKE SETPOINTS

    public static final double intakeForwardPower = 0.75;
    public static final double intakeReversePower = -0.75;

    /// LAUNCHER SETPOINTS
    public static final int launcherClose = 1500;
    public static final int launcherFar = 2000;
    public static final double DEFAULT_RPM = 1500.0;
    public static final double LAUNCH_ZONE_MID_RPM = 2000.0; // ~3.5 ft
    public static final double LAUNCH_ZONE_FAR_RPM = 2200.0; // ~5.5 ft
    public static final double LAUNCH_ZONE_FAR_FAR_RPM = 2600.0; // >8 ft
    public static final double FLYWHEEL_TOLERANCE_RPM = 75.0;
    public static final double LAUNCHER_GEAR_REDUCTION = 16.0 / 24.0; // motor:flywheel = 2:3

    /// LAUNCHER PIDF (base gains in motor units)
    // Free speed: 6000 rpm = 100 rps → 2,800 ticks/s (28 tpr encoder)
    // REV PIDF F = 32767 / maxTicksPerSecond
    // Bump the base gains to improve spin-up authority after gear scaling and
    // compensate for wheel inertia; tune at the motor side, then scale by the
    // reduction before applying to the flywheel velocity loop.
    public static final double LAUNCHER_F = 40.0;    // Stronger feedforward to reach setpoint
    public static final double LAUNCHER_P = 0.002;    // Tighter proportional correction // original 12
    public static final double LAUNCHER_I = 0.0001;    // Small I to clean steady-state error //original 0.05
    public static final double LAUNCHER_D = 0.001;     // Slightly more damping for step changes  //original 0.6

    ///  SPINDEXER SETPOINTS
    public static final double spindexerStart = 0.76;
    public static final double spindexer1 = 0.76;
    public static final double spindexer2 = 0.39;
    public static final double spindexer3 = 0.015;

    /// TURRET HOOD POSITIONS
    public static final double hoodMinimum = 0.0;
    public static final double hoodMaximum = 1.0;

    ///  TURRET POSITIONS
    public static final int turretHome = 0;
    public static final int turret_MIN = -1025;  //counter-clockwise from above starting facing opposite intake
    public static final int turret_MAX = 975;  //clockwise from above starting facing opposite intake
    public static final int turret_OBELISK_LEFT_LIMIT = -600;
    public static final int turret_OBELISK_RIGHT_LIMIT = 600;
    public static final double turret_FAR_AIM_DISTANCE_FEET = 8.0;
    public static final double turret_FAR_AIM_ADJUST_BLUE = -0.05; // aim left when distance exceeds threshold
    public static final double turret_FAR_AIM_ADJUST_RED = 0.05; // aim right when distance exceeds threshold

    ///  KICKER POSITIONS
    public static final double kickerDown = 0.0;
    public static final double kickerUp = 1.0;

    /// COLOR SENSOR
    public static final double COLOR_SENSOR_PURPLE_RATIO = 1.05; // Blue must exceed red and green by this factor
    public static final double COLOR_SENSOR_GREEN_BLUE_RATIO = 1.20; // Green must exceed blue by this factor
    public static final double COLOR_SENSOR_GREEN_RED_RATIO = 1.80;  // Green must exceed red by this factor
    // Tuned against field samples (purple B:G ≈ 1.2–1.6, green G:B ≈ 1.2 and G:R ≈ 2.4)
    // to keep registering balls even when the sensor happens to look into a hole
    // instead of solid plastic.
    public static final double COLOR_SENSOR_DETECTION_DISTANCE_MM = 55.0;

    ///AUTONOMOUS SETPOINTS

}