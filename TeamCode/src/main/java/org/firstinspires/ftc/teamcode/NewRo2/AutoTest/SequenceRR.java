package org.firstinspires.ftc.teamcode.NewRo2.AutoTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;

/**
 * SequenceRR:
 * - Scans Limelight once to store pattern (21=GPP, 22=PGP, 23=PPG)
 * - Runs the sequence using ONLY color4 verification (via TouchRev3)
 * - Keeps revolver PID running in the background
 * - DOES NOT use touch sensor (auto controls everything)
 *
 * Controls:
 * - Just press START. It will scan, then shoot.
 * - You can set RUN_TIMES > 1 to run sequence multiple times (debug).
 */
@Config
@Autonomous(name = "SequenceRR", group = "NEWRO")
public class SequenceRR extends LinearOpMode {

    // How many times to run the sequence (normally 1)
    public static int RUN_TIMES = 1;

    // Optional: if you want a small pause after scan before shooting
    public static double AFTER_SCAN_WAIT_SEC = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        TouchRev3 seq = new TouchRev3(hardwareMap);

        // If you want to tweak defaults from the OpMode side:
        // seq.LIMELIGHT_PIPELINE = 9;
        // seq.SHOOT_VEL = 3500;
        // seq.FIRST_REVUP_SEC = 1.0;
        // seq.VALIDATE_DELAY_SEC = 0.4;
        // seq.ARM_UP_SEC = 0.2;
        // seq.ARM_DOWN_SEC = 0.2;

        waitForStart();
        if (isStopRequested()) return;

        // Build "run sequence N times"
        Action runMany = buildRunMany(seq);

        Actions.runBlocking(
                new ParallelAction(
                        // Background: PID for revolver always running
                        seq.updatePID(),

                        // Safety: ensure touch sensor logic is OFF (auto controls it)
                        seq.disableSensor(),

                        // Foreground: scan then run sequence(s)
                        new SequentialAction(

                                // Optional pause
                                new SleepAction(AFTER_SCAN_WAIT_SEC),

                                // Run sequence(s)
                                runMany
                        )
                )
        );
    }

    private Action buildRunMany(TouchRev3 seq) {
        // RoadRunner needs explicit nesting; we’ll construct a sequential chain.
        SequentialAction chain = new SequentialAction();

        for (int i = 0; i < RUN_TIMES; i++) {
            chain = new SequentialAction(
                    chain,
                    new DebugPacketAction("SequenceRun", i + 1),
                    seq.runSequence3ShotsNoShooter()
            );
        }

        return chain;
    }

    /**
     * Tiny action to tag the packet so you can see iteration boundaries in dashboard logs.
     */
    private static class DebugPacketAction implements Action {
        private final String key;
        private final int value;
        private boolean done = false;

        DebugPacketAction(String key, int value) {
            this.key = key;
            this.value = value;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!done) {
                packet.put(key, value);
                done = true;
            }
            return false;
        }
    }
}
