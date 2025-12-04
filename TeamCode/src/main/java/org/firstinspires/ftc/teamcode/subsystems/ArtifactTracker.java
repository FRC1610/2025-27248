package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator.LEDColors;

/**
 * Utility to classify the contents of each spindexer position using three
 * color+distance sensors and drive the rear RGB indicators to match what is
 * detected.
 */
public class ArtifactTracker {

    public enum SlotStatus {
        VACANT,
        GREEN,
        PURPLE
    }

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final SlotStatus[] slotStatuses = new SlotStatus[]{SlotStatus.VACANT, SlotStatus.VACANT, SlotStatus.VACANT};
    private SlotStatus[] lastLoggedStatuses = null;

    public ArtifactTracker(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public SlotStatus[] getSlotStatuses() {
        return slotStatuses.clone();
    }

    public SlotStatus getSlotStatus(int index) {
        if (index < 0 || index >= slotStatuses.length) {
            throw new IndexOutOfBoundsException("Slot index must be between 0 and 2");
        }
        return slotStatuses[index];
    }

    /**
     * Refresh classifications and LED indicators for all three spindexer
     * positions. Intended to be invoked once per TeleOp loop.
     */
    public void update() {
        slotStatuses[0] = evaluateSensor(robot.color1, robot.distance1, robot.rearRGB1, 0);
        slotStatuses[1] = evaluateSensor(robot.color2, robot.distance2, robot.rearRGB2, 1);
        slotStatuses[2] = evaluateSensor(robot.color3, robot.distance3, robot.rearRGB3, 2);

        maybeLogStatusChange();

        telemetry.addData("Spindexer Slot 1", slotStatuses[0]);
        telemetry.addData("Spindexer Slot 2", slotStatuses[1]);
        telemetry.addData("Spindexer Slot 3", slotStatuses[2]);
    }

    private SlotStatus evaluateSensor(ColorSensor colorSensor, DistanceSensor distanceSensor, rgbIndicator indicator, int slot) {
        if (colorSensor == null || distanceSensor == null) {
            if (indicator != null) {
                indicator.setColor(LEDColors.OFF);
            }
            return SlotStatus.VACANT;
        }

        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        if (Double.isNaN(distance) || distance > Constants.COLOR_SENSOR_DETECTION_DISTANCE_MM) {
            if (indicator != null) {
                indicator.setColor(LEDColors.OFF);
            }
            return SlotStatus.VACANT;
        }

        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        SlotStatus status;
        if (blue >= Constants.COLOR_SENSOR_PURPLE_RATIO * green && blue >= Constants.COLOR_SENSOR_PURPLE_RATIO * red) {
            status = SlotStatus.PURPLE;
        } else if (green >= Constants.COLOR_SENSOR_GREEN_RATIO * blue) {
            status = SlotStatus.GREEN;
        } else {
            status = SlotStatus.VACANT;
        }

        if (indicator != null) {
            switch (status) {
                case PURPLE:
                    indicator.setColor(LEDColors.VIOLET);
                    break;
                case GREEN:
                    indicator.setColor(LEDColors.GREEN);
                    break;
                default:
                    indicator.setColor(LEDColors.OFF);
            }
        }

        telemetry.addData(String.format("Slot %d RGB", slot + 1), "R: %.0f, G: %.0f, B: %.0f, D: %.1fmm", red, green, blue, distance);
        return status;
    }

    private void maybeLogStatusChange() {
        boolean changed = lastLoggedStatuses == null
                || lastLoggedStatuses.length != slotStatuses.length;
        if (!changed) {
            for (int i = 0; i < slotStatuses.length; i++) {
                if (lastLoggedStatuses[i] != slotStatuses[i]) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed) {
            RobotLog.ii("ArtifactTracker", "Detected Artifacts: %s, %s, %s",
                    slotStatuses[0], slotStatuses[1], slotStatuses[2]);
            lastLoggedStatuses = slotStatuses.clone();
        }
    }
}
