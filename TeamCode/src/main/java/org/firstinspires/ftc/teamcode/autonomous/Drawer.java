package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.field.FieldManager;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class Drawer {
    /// CONSTANTS
    private static final double ROBOT_SIZE = 17.5;
    private static final double ROBOT_HALF_SIZE = ROBOT_SIZE/2;
    private static final double HEADING_OFFSET = 4.5;
    private static final double HEADING_LENGTH = 9.0;
    private static final Style STYLE = new Style("", "#FABF35", 0.75);

    private final FieldManager field;
    private final Follower follower;

    Drawer(FieldManager field, Follower follower) {
        this.field = field;
        this.follower = follower;
    }

    private boolean isValidPose(Pose pose) {
        return pose != null
                && !Double.isNaN(pose.getX())
                && !Double.isNaN(pose.getY())
                && !Double.isNaN(pose.getHeading());
    }

    private void drawRobot(Pose pose) {
        double x = pose.getX() - ROBOT_HALF_SIZE;
        double y = pose.getY() - ROBOT_HALF_SIZE;
        field.moveCursor(x, y);
        field.rect(ROBOT_SIZE, ROBOT_SIZE);
    }

    private void drawHeading(Pose pose) {
        double headingX = Math.cos(pose.getHeading());
        double headingY = Math.sin(pose.getHeading());

        double x1 = pose.getX() + headingX * HEADING_OFFSET;
        double y1 = pose.getY() + headingY * HEADING_OFFSET;
        double x2 = pose.getX() + headingX * HEADING_LENGTH;
        double y2 = pose.getY() + headingY * HEADING_LENGTH;

        field.moveCursor(x1, y1);
        field.line(x2, y2);
    }

    public void draw() {
        Pose pose = follower.getPose();
        if (!isValidPose(pose)) {
            return;
        }

        field.setStyle(STYLE);
        drawRobot(pose);
        drawHeading(pose);
        field.update();
    }
}
