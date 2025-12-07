package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Coordinates intake actions with the spindexer and ArtifactTracker color
 * sensors. The controller keeps at least one open slot pointed toward the
 * intake by rotating the spindexer when both visible slots are occupied.
 */
public class IntakeController {

    private final RobotHardware robot;
    private final ArtifactTracker artifactTracker;
    private final Telemetry telemetry;

    private boolean spindexerFull = false;

    private final double[] spindexerPositions = new double[]{Constants.spindexer1, Constants.spindexer2};
    private int spindexerIndex = 0;

    public IntakeController(RobotHardware robot, ArtifactTracker artifactTracker, Telemetry telemetry) {
        this.robot = robot;
        this.artifactTracker = artifactTracker;
        this.telemetry = telemetry;
        syncSpindexerIndex();
    }

    /**
     * Refresh sensor classifications and rotate the spindexer whenever both
     * slots facing the intake are occupied. When all three slots contain
     * artifacts, the spindexer is returned to position 1.
     */
    public void update() {
        artifactTracker.update();
        ArtifactTracker.SlotStatus[] slots = artifactTracker.getSlotStatuses();

        spindexerFull = allSlotsFilled(slots);
        if (spindexerFull) {
            moveToPosition(0);
            telemetry.addLine("Spindexer full; returned to position 1");
            return;
        }

        syncSpindexerIndex();
        if (spindexerIndex == 0) {
            boolean hiddenSlotVacant = slots[2] == ArtifactTracker.SlotStatus.VACANT;
            if (hiddenSlotVacant && slotsFilled(slots, 0, 1)) {
                moveToPosition(1);
                telemetry.addLine("Slots 1 & 2 full; rotating to position 2");
            }
        } else {
            boolean hiddenSlotVacant = slots[0] == ArtifactTracker.SlotStatus.VACANT;
            if (hiddenSlotVacant && slotsFilled(slots, 1, 2)) {
                moveToPosition(0);
                telemetry.addLine("Slots 2 & 3 full; rotating to position 1");
            }
        }

        telemetry.addData("Spindexer Facing", spindexerIndex + 1);
    }

    public boolean isSpindexerFull() {
        return spindexerFull;
    }

    private boolean allSlotsFilled(ArtifactTracker.SlotStatus[] slots) {
        for (ArtifactTracker.SlotStatus status : slots) {
            if (status == ArtifactTracker.SlotStatus.VACANT) {
                return false;
            }
        }
        return true;
    }

    private boolean slotsFilled(ArtifactTracker.SlotStatus[] slots, int firstIndex, int secondIndex) {
        return slots[firstIndex] != ArtifactTracker.SlotStatus.VACANT
                && slots[secondIndex] != ArtifactTracker.SlotStatus.VACANT;
    }

    private void syncSpindexerIndex() {
        double current = robot.spindexerPos;
        if (current == Constants.spindexer2) {
            spindexerIndex = 1;
        } else {
            spindexerIndex = 0;
        }
    }

    private void moveToPosition(int index) {
        index = Math.max(0, Math.min(index, spindexerPositions.length - 1));
        spindexerIndex = index;
        robot.spindexerPos = spindexerPositions[index];
        robot.spindexer.setPosition(robot.spindexerPos);
    }
}
