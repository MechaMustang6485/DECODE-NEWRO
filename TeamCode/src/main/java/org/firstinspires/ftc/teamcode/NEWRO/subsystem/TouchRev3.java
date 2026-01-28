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

    // =========================
    // CONFIG (REVOLVER)
    // =========================
    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = MAX_SLOTS * TICKS_PER_SLOT; // 288

    // Common positions
    public static int POS_INTAKE = 0;
    public static int POS_SHOOT0 = 48;
    public static int POS_SHOOT1 = 144;
    public static int POS_SHOOT2 = 240;

    // Tolerances / timeouts
    public static int AT_TARGET_TOL = 8;
    public static double MOVE_TIMEOUT_SEC = 1.25;
    private int lastSequenceId = -1;

    // =========================
    // CONFIG (SEQUENCE)
    // =========================
    public static double VALIDATE_DELAY_SEC = 0.4; // wait at shoot pos to read color4
    public static double ARM_UPDOWN_SEC = 0.2;     // arm up, then down
    public static double SEARCH_TIMEOUT_SEC = 1.2; // per shot search budget (fast)
    public static int MAX_JUMPS_PER_SHOT = 6;      // how many slot-changes allowed before fallback

    // Color thresholds (same as your processor)
    public static float GREEN_H_MIN = 100;
    public static float GREEN_H_MAX = 180;
    public static float PURPLE_H_MIN = 181;
    public static float PURPLE_H_MAX = 255;

    // =========================
    // CONFIG (LIMELIGHT)
    // =========================
    public static int LIMELIGHT_PIPELINE = 9;
    public static int POLL_HZ = 100;

    // =========================
    // CONFIG (ARM)
    // =========================
    public static double ARM_DOWN = 0.0;
    public static double ARM_UP = 0.3;

    // =========================
    // HARDWARE
    // =========================
    private final DcMotorEx revolver;
    private final TouchSensor touch;
    private final Limelight3A limelight;
    private final NormalizedColorSensor color4;
    private final Servo arm;

    // =========================
    // REV STATE
    // =========================
    private int targetPosition = 0;
    private int currentSlotCount = 0;
    private boolean lastButtonState = false;
    private boolean isSensorEnabled = false;

    private IMU imu;

    // =========================
    // LIMELIGHT STATE
    // =========================
    public enum Pattern { GPP, PGP, PPG }
    private volatile Pattern desiredPattern = Pattern.GPP;
    private volatile int lastFid = -1;

    // =========================
    // SEQUENCE STATE (internal)
    // =========================
    private boolean sequenceRunning = false;

    public enum BallColor { GREEN, PURPLE, UNKNOWN }

    public TouchRev3(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        arm = hardwareMap.get(Servo.class, "arm");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.setPollRateHz(POLL_HZ);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        color4.setGain(12);

        arm.setPosition(ARM_DOWN);

        targetPosition = 0;
        currentSlotCount = 0;
        lastButtonState = false;
        isSensorEnabled = false;
        sequenceRunning = false;
    }

    // =========================================================
    // BACKGROUND PID + OPTIONAL TOUCH LOADING (run in ParallelAction ALWAYS)
    // =========================================================
    public Action updatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Touch loading only when enabled AND not running sequence
                if (isSensorEnabled && !sequenceRunning) {
                    boolean isPressed = touch.isPressed();
                    if (isPressed && !lastButtonState && currentSlotCount < MAX_SLOTS) {
                        currentSlotCount++;
                        targetPosition = clamp(currentSlotCount * TICKS_PER_SLOT, 0, MAX_POSITION);
                    }
                    lastButtonState = isPressed;
                }

                // PID Control (your existing PIDClassForAuto)
                double pwr = PIDClassForAuto.returnRevPID(targetPosition, revolver.getCurrentPosition());

                // Clamp to prevent freakouts
                if (pwr > 0.6) pwr = 0.6;
                if (pwr < -0.6) pwr = -0.6;

                // Stop when close enough
                if (Math.abs(targetPosition - revolver.getCurrentPosition()) <= AT_TARGET_TOL) {
                    revolver.setPower(0);
                } else {
                    revolver.setPower(pwr);
                }

                // Telemetry
                packet.put("Rev Sensor Active", isSensorEnabled);
                packet.put("Rev SlotCount", currentSlotCount);
                packet.put("Rev Target", targetPosition);
                packet.put("Rev Actual", revolver.getCurrentPosition());
                packet.put("Rev Pwr", pwr);

                packet.put("LL Fid", lastFid);
                packet.put("LL Pattern", desiredPattern.toString());

                packet.put("Seq Running", sequenceRunning);

                return true;
            }
        };
    }

    // =========================================================
    // LIMELIGHT SCAN ACTION (call whenever; updates desiredPattern)
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

    public Pattern getDesiredPattern() {
        return desiredPattern;
    }

    public int getLastFid() {
        return lastFid;
    }

    public enum S {
        INIT,
        MOVE_TO_SHOOT,
        WAIT_VALIDATE,
        READ_VALIDATE,
        ARM_UP,
        ARM_DOWN,
        NEXT_SHOT,
        RETURN_INTAKE,
        DONE
    }

    // =========================================================
    // SEQUENCE ACTION (NO SHOOTER CONTROL — RR DOES THAT)
    // Uses ONLY color4 to verify at SHOOT positions.
    // If it can’t match quickly -> fallback shoots all remaining.
    // =========================================================
    public Action runSequence3ShotsNoShooter() {
        return new Action() {

            S state = S.INIT;

            ElapsedTime stateTimer;
            ElapsedTime moveTimer;
            ElapsedTime searchTimer;

            int shotsDone = 0;

            int currentShootPos = POS_SHOOT0; // start here unless already near a shoot pos
            int jumpsThisShot = 0;
            boolean fallbackShootAll = false;

            BallColor expected = BallColor.UNKNOWN;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                if (stateTimer == null) {
                    stateTimer = new ElapsedTime();
                    moveTimer = new ElapsedTime();
                    searchTimer = new ElapsedTime();
                }

                sequenceRunning = true;

                p.put("SeqState", state.toString());
                p.put("ShotsDone", shotsDone);
                p.put("Fallback", fallbackShootAll);
                p.put("Jumps", jumpsThisShot);
                p.put("Pattern", desiredPattern.toString());

                switch (state) {

                    case INIT: {
                        // Pick nearest shoot slot as starting point to avoid weird jumps
                        currentShootPos = nearestShootPos(revolver.getCurrentPosition());
                        setTargetInternal(currentShootPos);

                        shotsDone = 0;
                        jumpsThisShot = 0;
                        fallbackShootAll = false;

                        expected = expectedForShot(desiredPattern, shotsDone);

                        moveTimer.reset();
                        searchTimer.reset();
                        state = S.MOVE_TO_SHOOT;
                        break;
                    }

                    case MOVE_TO_SHOOT: {
                        // Wait until at target OR timeout
                        if (atTarget(currentShootPos) || moveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            stateTimer.reset();
                            state = S.WAIT_VALIDATE;
                        }
                        break;
                    }

                    case WAIT_VALIDATE: {
                        if (stateTimer.seconds() >= VALIDATE_DELAY_SEC) {
                            state = S.READ_VALIDATE;
                        }
                        break;
                    }

                    case READ_VALIDATE: {
                        BallColor seen = readColor4();
                        p.put("Seen", seen.toString());
                        p.put("Hue", getHue(color4));
                        p.put("Expected", expected.toString());

                        // If we already gave up, just shoot whatever is here
                        if (fallbackShootAll) {
                            stateTimer.reset();
                            state = S.ARM_UP;
                            break;
                        }

                        // If expected is UNKNOWN (shouldn't happen), just fallback
                        if (expected == BallColor.UNKNOWN) {
                            fallbackShootAll = true;
                            stateTimer.reset();
                            state = S.ARM_UP;
                            break;
                        }

                        // If it matches -> shoot
                        if (seen == expected) {
                            stateTimer.reset();
                            state = S.ARM_UP;
                            break;
                        }

                        // Not a match / unknown -> search next slot quickly
                        jumpsThisShot++;

                        boolean searchTimedOut = (searchTimer.seconds() >= SEARCH_TIMEOUT_SEC);
                        boolean jumpsExceeded = (jumpsThisShot >= MAX_JUMPS_PER_SHOT);

                        if (searchTimedOut || jumpsExceeded) {
                            // Give up: shoot all remaining, stop trying to pattern-match
                            fallbackShootAll = true;
                            stateTimer.reset();
                            state = S.ARM_UP;
                            break;
                        }

                        // Jump to next shoot slot (wrap among 48/144/240)
                        currentShootPos = nextShootPos(currentShootPos);
                        setTargetInternal(currentShootPos);

                        moveTimer.reset();
                        state = S.MOVE_TO_SHOOT;
                        break;
                    }

                    case ARM_UP: {
                        // Arm up (instant set), then wait ARM_UPDOWN_SEC
                        arm.setPosition(ARM_UP);
                        if (stateTimer.seconds() >= ARM_UPDOWN_SEC) {
                            stateTimer.reset();
                            state = S.ARM_DOWN;
                        }
                        break;
                    }

                    case ARM_DOWN: {
                        arm.setPosition(ARM_DOWN);
                        if (stateTimer.seconds() >= ARM_UPDOWN_SEC) {
                            state = S.NEXT_SHOT;
                        }
                        break;
                    }

                    case NEXT_SHOT: {
                        shotsDone++;

                        if (shotsDone >= 3) {
                            setTargetInternal(POS_INTAKE);
                            moveTimer.reset();
                            state = S.RETURN_INTAKE;
                            break;
                        }

                        // Go to next slot for next ball
                        currentShootPos = nextShootPos(currentShootPos);
                        setTargetInternal(currentShootPos);

                        // reset per-shot search
                        jumpsThisShot = 0;
                        searchTimer.reset();
                        expected = expectedForShot(desiredPattern, shotsDone);

                        moveTimer.reset();
                        stateTimer.reset();
                        state = S.MOVE_TO_SHOOT;
                        break;
                    }

                    case RETURN_INTAKE: {
                        if (atTarget(POS_INTAKE) || moveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            state = S.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        arm.setPosition(ARM_DOWN);
                        sequenceRunning = false;
                        return false;
                    }
                }

                return true;
            }
        };
    }

    // =========================================================
    // ACTIONS (KEEPED FROM TOUCHREV2 + FIXED)
    // =========================================================
    public Action resetRevolver() {
        return packet -> {
            targetPosition = 0;
            currentSlotCount = 0;
            lastButtonState = false;
            return false;
        };
    }

    public Action enableSensor() {
        return packet -> {
            isSensorEnabled = true;
            lastButtonState = false;
            return false;
        };
    }

    public Action disableSensor() {
        return packet -> {
            isSensorEnabled = false;
            lastButtonState = false;
            return false;
        };
    }

    public Action resetHardwareEncoder() {
        return packet -> {
            revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetPosition = 0;
            currentSlotCount = 0;
            lastButtonState = false;
            return false;
        };
    }

    public Action setTarget(int pos) {
        return new SetTargetAction(pos);
    }

    private class SetTargetAction implements Action {
        private final int pos;
        SetTargetAction(int position) { this.pos = position; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setTargetInternal(pos);
            // keep slot count consistent (0..3)
            currentSlotCount = Math.round((float) targetPosition / TICKS_PER_SLOT);
            return false;
        }
    }

    // =========================================================
    // INTERNAL HELPERS
    // =========================================================
    private void setTargetInternal(int pos) {
        targetPosition = clamp(pos, 0, MAX_POSITION);
    }

    private boolean atTarget(int target) {
        return Math.abs(revolver.getCurrentPosition() - target) <= AT_TARGET_TOL;
    }

    private int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private int nearestShootPos(int ticks) {
        int[] poses = { POS_SHOOT0, POS_SHOOT1, POS_SHOOT2 };
        int best = poses[0];
        int bestErr = Math.abs(ticks - poses[0]);
        for (int i = 1; i < poses.length; i++) {
            int e = Math.abs(ticks - poses[i]);
            if (e < bestErr) {
                bestErr = e;
                best = poses[i];
            }
        }
        return best;
    }

    private int nextShootPos(int curShootPos) {
        if (curShootPos == POS_SHOOT0) return POS_SHOOT1;
        if (curShootPos == POS_SHOOT1) return POS_SHOOT2;
        return POS_SHOOT0;
    }

    private BallColor expectedForShot(Pattern p, int shotIdx) {
        // shotIdx: 0..2
        if (p == Pattern.GPP) {
            return (shotIdx == 0) ? BallColor.GREEN : BallColor.PURPLE;
        } else if (p == Pattern.PGP) {
            return (shotIdx == 1) ? BallColor.GREEN : BallColor.PURPLE;
        } else { // PPG
            return (shotIdx == 2) ? BallColor.GREEN : BallColor.PURPLE;
        }
    }

    // =========================================================
    // COLOR4 ONLY
    // =========================================================
    private BallColor readColor4() {
        float h = getHue(color4);
        if (h >= GREEN_H_MIN && h <= GREEN_H_MAX) return BallColor.GREEN;
        if (h >= PURPLE_H_MIN && h <= PURPLE_H_MAX) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    private float getHue(NormalizedColorSensor sensor) {
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }
}
