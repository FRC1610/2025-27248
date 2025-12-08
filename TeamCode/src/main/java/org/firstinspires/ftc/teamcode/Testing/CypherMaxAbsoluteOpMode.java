package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivers.CypherMaxAbsoluteEncoder;

/**
 * Simple OpMode that shows the Studica Cypher Max absolute PWM encoder on telemetry
 * during both init and tele-op. Wire the encoder's PWM signal to a digital input
 * and set that input's configuration name to {@code cypherEncoder} (or update the
 * string below to match your configuration).
 */
@TeleOp(name = "Cypher Max Absolute Encoder", group = "Sensor")
public class CypherMaxAbsoluteOpMode extends LinearOpMode {

    private static final String ENCODER_DEVICE_NAME = "cypherEncoder";
    private CypherMaxAbsoluteEncoder encoder;

    @Override
    public void runOpMode() {
        encoder = new CypherMaxAbsoluteEncoder(hardwareMap, ENCODER_DEVICE_NAME);

        telemetry.setMsTransmissionInterval(50);

        while (opModeInInit() && !isStopRequested()) {
            encoder.update();
            sendTelemetry();
            sleep(10);
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            encoder.update();
            sendTelemetry();
            sleep(10);
        }
    }

    private void sendTelemetry() {
        telemetry.addData("Duty Cycle", "%.3f", encoder.getDutyCycle());
        telemetry.addData("Counts", "%.1f / 4095", encoder.getPositionCounts());
        telemetry.addData("Degrees", "%.2f", encoder.getPositionDegrees());
        telemetry.addData("Zeroed Counts", "%.1f / 4095", encoder.getZeroedPositionCounts());
        telemetry.addData("Zeroed Degrees", "%.2f", encoder.getZeroedPositionDegrees());
        telemetry.addData("Frame Valid", encoder.isValid());
        telemetry.update();
    }
}
