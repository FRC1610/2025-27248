package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3;

/**
 * Simple TeleOp to verify the OctoQuad is reporting the Axon servo's absolute position.
 * Expects the servo signal connected to OctoQuad port 0.
 */
@TeleOp(name = "Octo Test", group = "Test")
public class OctoTest extends LinearOpMode {

    private static final int SERVO_PORT = 0;

    private OctoQuadFWv3 octoQuad;
    private Servo spindexer;
    private double spindexerPos = Constants.spindexerStart;

    @Override
    public void runOpMode() {
        octoQuad = hardwareMap.get(OctoQuadFWv3.class, "octoQuad");
        octoQuad.setChannelBankConfig(OctoQuadFWv3.ChannelBankConfig.ALL_PULSE_WIDTH);
        octoQuad.setSingleChannelPulseWidthTracksWrap(SERVO_PORT, false);
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        spindexer.setPosition(spindexerPos);

        waitForStart();

        while (opModeIsActive()) {
            // Spindexer manual control matching Competition TeleOp (gamepad2)
            if (gamepad2.b) {
                spindexerPos = Constants.spindexer1;
            } else if (gamepad2.y) {
                spindexerPos = Constants.spindexer2;
            } else if (gamepad2.x) {
                spindexerPos = Constants.spindexer3;
            }
            spindexer.setPosition(spindexerPos);

            OctoQuadFWv3.EncoderDataBlock dataBlock = octoQuad.readAllEncoderData();
            int pulseWidthMicros = dataBlock.positions[SERVO_PORT];

            telemetry.addData("CRC OK", dataBlock.isDataValid());
            telemetry.addData("Pulse Width (us)", pulseWidthMicros);

            // With pulse-width mode disabled from wrap tracking, the raw pulse width should stay
            // within the configured servo range. Clamp so telemetry stays readable if noise occurs.
            int clampedPulse = Range.clip(pulseWidthMicros, OctoQuadFWv3.MIN_PULSE_WIDTH_US, OctoQuadFWv3.MAX_PULSE_WIDTH_US);
            telemetry.addData("Clamped (us)", clampedPulse);
            telemetry.addData("Spindexer Pos", spindexerPos);
            telemetry.update();

            idle();
        }
    }
}
