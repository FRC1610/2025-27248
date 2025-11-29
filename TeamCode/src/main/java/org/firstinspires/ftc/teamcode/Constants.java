package org.firstinspires.ftc.teamcode;

public class Constants {

    ///INTAKE SETPOINTS

    public static final double intakeForwardPower = 0.75;
    public static final double intakeReversePower = -0.75;

    /// LAUNCHER SETPOINTS
    public static final int launcherClose = 1500;
    public static final int launcherFar = 2000;
    public static final double DEFAULT_RPM = 1500.0;
    public static final double LAUNCH_ZONE_MID_RPM = 2000.0; // ~3.5 ft
    public static final double LAUNCH_ZONE_FAR_RPM = 2200.0; // ~5.5 ft
    public static final double FLYWHEEL_TOLERANCE_RPM = 50.0;
    public static final double LAUNCHER_GEAR_REDUCTION = 24.0 / 16.0; // motor:flywheel = 1.5:1

    /// LAUNCHER PIDF (6000 RPM Yellow Jacket, 16:24 reduction)
    // Free speed: 6000 rpm = 100 rps → 2,800 ticks/s (28 tpr encoder)
    // REV PIDF F = 32767 / maxTicksPerSecond
    public static final double LAUNCHER_F = 11.7;    // 32767 / 2800, adjust after testing
    public static final double LAUNCHER_P = 10.0;    // Start aggressive enough to fight droop
    public static final double LAUNCHER_I = 0.0;     // Keep zero to avoid windup during bursts
    public static final double LAUNCHER_D = 0.5;     // Light damping for overshoot/ringing

    ///  SPINDEXER SETPOINTS
    public static final double spindexerStart = 0.5;
    public static final double spindexer1 = 0.04;
    public static final double spindexer2 = 0.42;
    public static final double spindexer3 = 0.78;

    /// TURRET HOOD POSITIONS
    public static final double hoodMinimum = 0.0;
    public static final double hoodMaximum = 1.0;

    ///  TURRET POSITIONS
    public static final int turretHome = 0;
    public static final int turret_MIN = -750;  //counter-clockwise from above starting facing opposite intake
    public static final int turret_MAX = 500;  //clockwise from above starting facing opposite intake

    ///  KICKER POSITIONS
    public static final double kickerDown = 0.0;
    public static final double kickerUp = 1.0;

    /// COLOR SENSOR
    public static final int intakeColorRed = 4000;
    public static final int intakeColorGreen = 4000;
    public static final int intakeColorBlue = 4000;

    ///AUTONOMOUS SETPOINTS

}