package org.firstinspires.ftc.teamcode.NEWRO.subsystem;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;

import java.util.List;

@Config
public class TouchRev3 {
    // inside TouchRev3
    public boolean isTouchPressed() {
        return touchSensor.isPressed();
    }

    // =========================
    // HARDWARE
    // =========================
    private final DcMotorEx revolver;
    private final TouchSensor touchSensor; // kept for compatibility (NOT used by sequence)
    private final Servo arm;

    private final NormalizedColorSensor color4; // FINAL SAY at shooting slot
    private final Limelight3A limelight;
    private final IMU imu;

    // =========================
    // REVOLVER CONFIG
    // =========================
    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = MAX_SLOTS * TICKS_PER_SLOT; // 288

    public static int REV_HOME = 0;     // intake alignment
    public static int REV_SHOOT_BASE = 48; // shoot alignment base (slot0)

    // target position for PID
    private int targetPosition = 0;

    // Touch logic variables (kept from TouchRev2)
    private int currentSlotCount = 0;
    private boolean lastButtonState = false;
    private boolean isSensorEnabled = false;

    // =========================
    // ARM CONFIG
    // =========================
    public static double ARM_DOWN = 0.0;
    public static double ARM_UP = 0.3;

    // arm timing (you requested 0.2 up/down)
    public static double ARM_UP_TIME = 0.20;
    public static double ARM_DOWN_TIME = 0.20;

    // =========================
    // COLOR4 VALIDATION CONFIG
    // =========================
    // you asked: validation delay be 0.4 seconds
    public static double COLOR4_VALIDATE_DELAY = 0.40;

    // you also asked: wait 0.7 seconds per ball so it's sure
    public static double PER_BALL_SETTLE = 0.70;

    // retry limits for searching correct color
    public static int MAX_SEARCH_ATTEMPTS_PER_SHOT = 4;

    // =========================
    // LIMELIGHT PATTERN
    // 21=GPP, 22=PGP, 23=PPG
    // =========================
    public enum Pattern { GPP, PGP, PPG }
    private Pattern desiredPattern = Pattern.GPP;
    private int lastSequenceId = -1;

    // Limelight scanning tuning
    public static int LIMELIGHT_PIPELINE = 9;
    public static int LIMELIGHT_POLL_HZ = 100;
    public static double SCAN_TIME_SEC = 0.25; // short + snappy

    // =========================
    // SEQUENCE FSM
    // =========================
    private enum SeqState {
        IDLE,

        MOVE_TO_SHOOT_SLOT,
        WAIT_COLOR4,
        READ_COLOR4,

        ARM_UP,
        ARM_DOWN,
        POST_BALL_WAIT,

        NEXT_SHOT,

        DUMP_ALL_INIT,
        DUMP_ALL_MOVE,
        DUMP_ALL_ARM_UP,
        DUMP_ALL_ARM_DOWN,
        DUMP_ALL_POST,

        DONE,
        ABORT
    }

    // live state (per runSequence Action instance)
    // (stored inside the Action via fields, NOT static!)

    public TouchRev3(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor"); // kept
        arm = hardwareMap.get(Servo.class, "arm");

        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        color4.setGain(12);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(LIMELIGHT_POLL_HZ);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);

        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
        currentSlotCount = 0;

        arm.setPosition(ARM_DOWN);
    }

    // =========================================================
    // PID BACKGROUND ACTION (kept from TouchRev2)
    // =========================================================
    public Action updatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Touch sensor logic (ONLY if enabled)
                if (isSensorEnabled) {
                    boolean isPressed = touchSensor.isPressed();
                    if (isPressed && !lastButtonState && currentSlotCount < MAX_SLOTS) {
                        currentSlotCount++;
                        targetPosition = currentSlotCount * TICKS_PER_SLOT;
                    }
                    lastButtonState = isPressed;
                }

                // PID to targetPosition
                double pwr = PIDClassForAuto.returnRevPID(targetPosition, revolver.getCurrentPosition());
                revolver.setPower(pwr);

                packet.put("Rev Sensor Active", isSensorEnabled);
                packet.put("Rev Slot", currentSlotCount);
                packet.put("Rev Target", targetPosition);
                packet.put("Rev Actual", revolver.getCurrentPosition());
                packet.put("LL Pattern", desiredPattern.toString());
                packet.put("LL ID", lastSequenceId);

                return true; // always running
            }
        };
    }

    // =========================================================
    // ACTIONS KEPT FROM TouchRev2
    // =========================================================
    public Action resetRevolver() {
        return packet -> {
            targetPosition = 0;
            currentSlotCount = 0;
            return false;
        };
    }

    public Action enableSensor() {
        return packet -> {
            isSensorEnabled = true;
            return false;
        };
    }

    public Action disableSensor() {
        return packet -> {
            isSensorEnabled = false;
            return false;
        };
    }

    public Action resetHardwareEncoder() {
        return packet -> {
            revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetPosition = 0;
            currentSlotCount = 0;
            return false;
        };
    }

    public Action setTarget(int pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (pos > MAX_POSITION) targetPosition = MAX_POSITION;
                else if (pos < 0) targetPosition = 0;
                else targetPosition = pos;

                currentSlotCount = Math.round((float) targetPosition / TICKS_PER_SLOT);
                packet.put("SetTarget", targetPosition);
                return false;
            }
        };
    }

    // =========================================================
    // LIMELIGHT SCAN ACTION (run once, store pattern)
    // =========================================================
    public Action scanLimelightPattern() {
        return new Action() {

            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!init) {
                    init = true;

                    limelight.setPollRateHz(100);
                    limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
                    limelight.start();           // IMPORTANT
                    timer.reset();
                }

                // Update robot orientation for Limelight
                YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(ypr.getYaw(AngleUnit.RADIANS));

                LLResult result = limelight.getLatestResult();

                int bestId = -1;
                double bestArea = -1;

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null) {
                        for (LLResultTypes.FiducialResult tag : tags) {
                            int id = tag.getFiducialId();
                            if (id != 21 && id != 22 && id != 23) continue;

                            double area = tag.getTargetArea();
                            if (area > bestArea) {
                                bestArea = area;
                                bestId = id;
                            }
                        }
                    }
                }

                // === Store result if found ===
                if (bestId != -1) {
                    lastSequenceId = bestId;

                    if (bestId == 21) desiredPattern = Pattern.GPP;
                    else if (bestId == 22) desiredPattern = Pattern.PGP;
                    else desiredPattern = Pattern.PPG;

                    packet.put("LL_FOUND_ID", bestId);
                    packet.put("LL_PATTERN", desiredPattern.toString());

                    return false; // DONE
                }

                // === Timeout safety (don’t hang auto) ===
                if (timer.seconds() > 1.0) {
                    packet.put("LL_TIMEOUT", true);
                    packet.put("LL_LAST_ID", lastSequenceId);
                    packet.put("LL_PATTERN", desiredPattern.toString());
                    return false;
                }

                // keep scanning
                packet.put("LL_SCANNING", true);
                packet.put("LL_TIME", timer.seconds());
                return true;
            }
        };
    }

    public class IntakerAction implements Action {

        private final DcMotor intake;
        private final TouchRev3 revolver;

        private boolean initialized = false;
        private boolean done = false;

        private int ballsLoaded = 0;
        private boolean lastTouchState = false;

        public int MAX_BALLS = 3;

        public IntakerAction(DcMotor intakeMotor, TouchRev3 rev) {
            this.intake = intakeMotor;
            this.revolver = rev;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                initialized = true;
                ballsLoaded = 0;
                lastTouchState = false;

                // Enable intake + touch logic
                intake.setPower(1.0);
                revolver.enableSensor().run(packet);

                packet.put("Intaker", "STARTED");
            }


            boolean touchPressed = revolver.isTouchPressed();

            // Rising edge detect
            if (touchPressed && !lastTouchState) {
                ballsLoaded++;
                packet.put("BallDetected", ballsLoaded);
            }
            lastTouchState = touchPressed;

            // Done after 3 balls
            if (ballsLoaded >= MAX_BALLS) {
                intake.setPower(0);
                revolver.disableSensor().run(packet);
                done = true;

                packet.put("Intaker", "DONE");
            }

            packet.put("BallsLoaded", ballsLoaded);
            return !done;
        }
    }


    // =========================================================
    // SEQUENCE ACTION (NO SHOOTER CONTROL)
    // =========================================================
    public Action runSequence() {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();

            private SeqState state = SeqState.IDLE;

            // Which shot we are on (0..2)
            private int shotIdx = 0;

            // Slot pointer for where we are trying to shoot (0..2)
            private int slotPtr = 0;

            // Attempts made to find expected color for this shot
            private int attempts = 0;

            // When pattern becomes impossible -> dump all remaining
            private boolean dumpAll = false;
            private int dumpIdx = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    init = true;
                    state = SeqState.MOVE_TO_SHOOT_SLOT;
                    timer.reset();

                    // Start at slot 0 shoot alignment by default
                    shotIdx = 0;
                    slotPtr = 0;
                    attempts = 0;
                    dumpAll = false;
                    dumpIdx = 0;

                    // move to first slot shoot pose
                    targetPosition = ticksForShootSlot(slotPtr);
                }

                packet.put("SeqState", state.toString());
                packet.put("ShotIdx", shotIdx);
                packet.put("SlotPtr", slotPtr);
                packet.put("Attempts", attempts);
                packet.put("DumpAll", dumpAll);
                packet.put("Pattern", desiredPattern.toString());

                switch (state) {

                    case MOVE_TO_SHOOT_SLOT: {
                        timer.reset();
                        state = SeqState.WAIT_COLOR4;
                        break;
                    }

                    case WAIT_COLOR4: {
                        // give sensor time to stabilize at shooting position
                        if (timer.seconds() >= COLOR4_VALIDATE_DELAY) {
                            state = SeqState.READ_COLOR4;
                        }
                        break;
                    }

                    case READ_COLOR4: {
                        if (dumpAll) {
                            state = SeqState.DUMP_ALL_INIT;
                            break;
                        }

                        BallColor seen = readColor4();
                        BallColor expected = expectedForShot(desiredPattern, shotIdx);

                        packet.put("Seen", seen.toString());
                        packet.put("Expected", expected.toString());

                        if (seen == expected) {
                            // Shoot this ball
                            arm.setPosition(ARM_UP);
                            timer.reset();
                            state = SeqState.ARM_UP;
                        } else {
                            // Wrong/unknown: search next slot
                            attempts++;

                            if (attempts >= MAX_SEARCH_ATTEMPTS_PER_SHOT) {
                                // Could not find expected -> switch to dump all remaining
                                dumpAll = true;
                                state = SeqState.DUMP_ALL_INIT;
                                break;
                            }

                            // Move forward to check next slot
                            slotPtr = wrapSlot(slotPtr + 1);
                            targetPosition = ticksForShootSlot(slotPtr);

                            timer.reset();
                            state = SeqState.MOVE_TO_SHOOT_SLOT;
                        }
                        break;
                    }

                    case ARM_UP: {
                        if (timer.seconds() >= ARM_UP_TIME) {
                            arm.setPosition(ARM_DOWN);
                            timer.reset();
                            state = SeqState.ARM_DOWN;
                        }
                        break;
                    }

                    case ARM_DOWN: {
                        if (timer.seconds() >= ARM_DOWN_TIME) {
                            timer.reset();
                            state = SeqState.POST_BALL_WAIT;
                        }
                        break;
                    }

                    case POST_BALL_WAIT: {
                        // you requested ~0.7s per ball to ensure the correct ball moved
                        if (timer.seconds() >= PER_BALL_SETTLE) {

                            // After successfully shooting a correct ball:
                            // backtrack one slot and try continuing (prevents “hole drifting forward”)
                            slotPtr = wrapSlot(slotPtr - 1);
                            targetPosition = ticksForShootSlot(slotPtr);

                            state = SeqState.NEXT_SHOT;
                        }
                        break;
                    }

                    case NEXT_SHOT: {
                        shotIdx++;
                        attempts = 0;

                        if (shotIdx >= 3) {
                            state = SeqState.DONE;
                        } else {
                            // go validate at current slotPtr
                            timer.reset();
                            state = SeqState.MOVE_TO_SHOOT_SLOT;
                        }
                        break;
                    }

                    // =========================
                    // DUMP ALL REMAINING (no color checks)
                    // =========================
                    case DUMP_ALL_INIT: {
                        // dump remaining shots as fast as possible
                        dumpIdx = shotIdx;
                        state = SeqState.DUMP_ALL_MOVE;

                        // dump starts from current slot pointer
                        targetPosition = ticksForShootSlot(slotPtr);
                        timer.reset();
                        break;
                    }

                    case DUMP_ALL_MOVE: {
                        // short settle then fire
                        if (timer.seconds() >= 0.15) { // shortened “dumpfallback” time
                            arm.setPosition(ARM_UP);
                            timer.reset();
                            state = SeqState.DUMP_ALL_ARM_UP;
                        }
                        break;
                    }

                    case DUMP_ALL_ARM_UP: {
                        if (timer.seconds() >= ARM_UP_TIME) {
                            arm.setPosition(ARM_DOWN);
                            timer.reset();
                            state = SeqState.DUMP_ALL_ARM_DOWN;
                        }
                        break;
                    }

                    case DUMP_ALL_ARM_DOWN: {
                        if (timer.seconds() >= ARM_DOWN_TIME) {
                            timer.reset();
                            state = SeqState.DUMP_ALL_POST;
                        }
                        break;
                    }

                    case DUMP_ALL_POST: {
                        if (timer.seconds() >= 0.25) { // short settle
                            dumpIdx++;
                            if (dumpIdx >= 3) {
                                state = SeqState.DONE;
                            } else {
                                // move to next slot and keep dumping
                                slotPtr = wrapSlot(slotPtr + 1);
                                targetPosition = ticksForShootSlot(slotPtr);
                                timer.reset();
                                state = SeqState.DUMP_ALL_MOVE;
                            }
                        }
                        break;
                    }

                    case DONE: {
                        // return to intake/home
                        targetPosition = REV_HOME;
                        arm.setPosition(ARM_DOWN);
                        return false;
                    }

                    case ABORT: {
                        targetPosition = REV_HOME;
                        arm.setPosition(ARM_DOWN);
                        return false;
                    }

                    default:
                        state = SeqState.DONE;
                        break;
                }

                return true;
            }
        };
    }

    // =========================================================
    // COLOR + PATTERN HELPERS
    // =========================================================
    private enum BallColor { GREEN, PURPLE, UNKNOWN }

    private BallColor readColor4() {
        float hue = getHue(color4);

        // same thresholds you’ve been using
        if (hue >= 100 && hue <= 180) return BallColor.GREEN;
        if (hue >= 181 && hue <= 255) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    private float getHue(NormalizedColorSensor sensor) {
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    private BallColor expectedForShot(Pattern p, int idx) {
        idx = Math.max(0, Math.min(2, idx));
        switch (p) {
            case GPP:
                return (idx == 0) ? BallColor.GREEN : BallColor.PURPLE;
            case PGP:
                return (idx == 1) ? BallColor.GREEN : BallColor.PURPLE;
            case PPG:
            default:
                return (idx == 2) ? BallColor.GREEN : BallColor.PURPLE;
        }
    }

    private int ticksForShootSlot(int slotIdx) {
        slotIdx = wrapSlot(slotIdx);
        return REV_SHOOT_BASE + (slotIdx * TICKS_PER_SLOT);
    }

    private int wrapSlot(int s) {
        int r = s % MAX_SLOTS;
        if (r < 0) r += MAX_SLOTS;
        return r;
    }

    // =========================================================
    // OPTIONAL: getters for telemetry/debug
    // =========================================================
    public Pattern getDesiredPattern() { return desiredPattern; }
    public int getLastSequenceId() { return lastSequenceId; }
    public int getTargetPosition() { return targetPosition; }
}
