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
import com.qualcomm.robotcore.hardware.CRServo;
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
    // REVOLVER CONFIG
    // =========================
    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = MAX_SLOTS * TICKS_PER_SLOT; // 288

    public static int POS_INTAKE = 0;
    public static int POS_SHOOT0 = 48;
    public static int POS_SHOOT1 = 144;
    public static int POS_SHOOT2 = 240;

    public static int AT_TARGET_TOL = 8;

    public double MOVE_TIMEOUT_SEC = 0.45;

    // =========================
    // COLOR4 CONFIG
    // =========================
    public static float GREEN_H_MIN = 100;
    public static float GREEN_H_MAX = 180;
    public static float PURPLE_H_MIN = 181;
    public static float PURPLE_H_MAX = 255;

    public double VALIDATE_DELAY_SEC = 0.40;      // wait at shoot pos before reading
    public double UNKNOWN_SCAN_SEC = 0.70;        // if UNKNOWN, keep sampling for this long

    // Only validate first 2 balls (3rd assumed correct)
    public static boolean ONLY_VALIDATE_FIRST_TWO = true;

    // Extra delay AFTER validation passes and BEFORE arm up (your “rev-up” window)
    public double PRE_SHOT_REV_DELAY_SEC = 0.50;

    // Arm timing
    public double ARM_UPDOWN_SEC = 0.20;

    // After a shot, give the next ball time to settle into the slot
    public double BALL_SETTLE_SEC = 0.70;

    // =========================
    // LIMELIGHT CONFIG
    // =========================
    public static int LIMELIGHT_PIPELINE = 9;
    public static int POLL_HZ = 100;
    public double LIMELIGHT_SCAN_TIMEOUT_SEC = 1.0;

    // =========================
    // TURRET CONFIG (optional)
    // =========================
    public double turretP = 0.02;
    public double turretD = 0.002;
    public double turretMaxPower = 0.5;
    public double turretMinPower = 0.05;
    public double turretTolerance = 0.5;

    public boolean turretLimits = true;
    public int turretMinEnc = -3_000_000;
    public int turretMaxEnc = 3_000_000;

    // =========================
    // ARM CONFIG
    // =========================
    public static double ARM_DOWN = 0.0;
    public static double ARM_UP = 0.3;

    // =========================
    // HARDWARE
    // =========================
    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;
    private final Limelight3A limelight;
    private final NormalizedColorSensor color4;
    private final Servo arm;

    // Turret (optional)
    public final CRServo turretServo;
    private final DcMotorEx turretEncoderMotor;

    private final IMU imu;

    // =========================
    // TOUCHREV2-STYLE STATE
    // =========================
    private int targetPosition = 0;
    private int currentSlotCount = 0;
    private boolean lastButtonState = false;
    private boolean isSensorEnabled = false;

    // =========================
    // LIMELIGHT STATE
    // =========================
    public enum Pattern { GPP, PGP, PPG }
    private volatile Pattern desiredPattern = Pattern.GPP;
    private volatile int lastSequenceId = -1;

    // =========================
    // SEQUENCE STATE (PERSISTENT FIELDS)
    // =========================
    private boolean sequenceRunning = false;

    private enum S {
        INIT,
        MOVE_TO_TARGET,
        WAIT_VALIDATE,
        READ_VALIDATE,
        PRE_SHOT_REV_DELAY,
        ARM_UP_STATE,
        ARM_DOWN_STATE,
        POST_SHOT_SETTLE,
        NEXT_SHOT,
        RETURN_INTAKE,
        DONE
    }

    private S seqState = S.INIT;

    private int shotsDone = 0;
    private int currentIdx = 0;
    private int targetIdx = 0;
    private int targetPos = POS_SHOOT0;

    private final boolean[] shotMask = new boolean[]{false, false, false};

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime unknownScanTimer = new ElapsedTime();

    private BallColor lastNonUnknownSeen = BallColor.UNKNOWN;

    private final int[] SHOOT_POS = new int[]{POS_SHOOT0, POS_SHOOT1, POS_SHOOT2};

    public enum BallColor { GREEN, PURPLE, UNKNOWN }

    // Turret PID runtime
    private double turretLastError = 0;
    private final ElapsedTime turretPidTimer = new ElapsedTime();

    // =========================
    // CONSTRUCTOR
    // =========================
    public TouchRev3(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        arm = hardwareMap.get(Servo.class, "arm");

        // Optional turret hardware (OpModeTurret names)
        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretEncoderMotor = hardwareMap.get(DcMotorEx.class, "Fl");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.setPollRateHz(POLL_HZ);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hub = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(hub));

        color4.setGain(12);
        arm.setPosition(ARM_DOWN);

        targetPosition = 0;
        currentSlotCount = 0;
        lastButtonState = false;
        isSensorEnabled = false;

        resetSequenceState();
    }

    // =========================
    // BACKGROUND PID (RUN IN ParallelAction ALWAYS)
    // =========================
    public Action updatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Touch loading ONLY when enabled and not sequencing
                if (isSensorEnabled && !sequenceRunning) {
                    boolean pressed = touchSensor.isPressed();
                    if (pressed && !lastButtonState && currentSlotCount < MAX_SLOTS) {
                        currentSlotCount++;
                        targetPosition = clamp(currentSlotCount * TICKS_PER_SLOT, 0, MAX_POSITION);
                    }
                    lastButtonState = pressed;
                }

                double pwr = PIDClassForAuto.returnRevPID(targetPosition, revolver.getCurrentPosition());

                // clamp
                if (pwr > 0.6) pwr = 0.6;
                if (pwr < -0.6) pwr = -0.6;

                if (Math.abs(targetPosition - revolver.getCurrentPosition()) <= AT_TARGET_TOL) {
                    revolver.setPower(0);
                } else {
                    revolver.setPower(pwr);
                }

                packet.put("Rev/SensorEnabled", isSensorEnabled);
                packet.put("Rev/SlotCount", currentSlotCount);
                packet.put("Rev/Target", targetPosition);
                packet.put("Rev/Actual", revolver.getCurrentPosition());
                packet.put("Seq/Running", sequenceRunning);
                packet.put("LL/LastID", lastSequenceId);
                packet.put("LL/Pattern", desiredPattern.toString());

                return true;
            }
        };
    }

    // =========================
    // TOUCHREV2 ACTIONS (KEPT)
    // =========================
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
        return packet -> {
            setTargetInternal(pos);
            currentSlotCount = Math.round((float) targetPosition / TICKS_PER_SLOT);
            return false;
        };
    }

    // =========================
    // LIMELIGHT: START/STOP
    // =========================
    public Action startLimelight() {
        return packet -> {
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.setPollRateHz(POLL_HZ);
            limelight.start();
            return false;
        };
    }

    public Action stopLimelight() {
        return packet -> {
            limelight.stop();
            return false;
        };
    }

    // =========================
    // LIMELIGHT: SCAN PATTERN (21/22/23)
    // =========================
    public Action scanLimelightPattern() {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    init = true;
                    limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
                    limelight.setPollRateHz(POLL_HZ);
                    limelight.start();
                    timer.reset();
                }

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

                if (bestId != -1) {
                    lastSequenceId = bestId;
                    if (bestId == 21) desiredPattern = Pattern.GPP;
                    else if (bestId == 22) desiredPattern = Pattern.PGP;
                    else desiredPattern = Pattern.PPG;

                    packet.put("LL/FOUND_ID", bestId);
                    packet.put("LL/PATTERN", desiredPattern.toString());
                    return false;
                }

                if (timer.seconds() > LIMELIGHT_SCAN_TIMEOUT_SEC) {
                    packet.put("LL/TIMEOUT", true);
                    packet.put("LL/LAST_ID", lastSequenceId);
                    packet.put("LL/PATTERN", desiredPattern.toString());
                    return false;
                }

                packet.put("LL/SCANNING", true);
                packet.put("LL/T", timer.seconds());
                return true;
            }
        };
    }

    public Pattern getDesiredPattern() { return desiredPattern; }
    public int getLastSequenceId() { return lastSequenceId; }

    // =========================
    // TURRET: TRACK (RUN FOREVER IN ParallelAction)
    // =========================
    public Action turretTrack() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    turretPidTimer.reset();
                    initialized = true;
                }
                runTurretLogic();

                packet.put("Turret/Enc", turretEncoderMotor.getCurrentPosition());
                packet.put("Turret/Pwr", turretServo.getPower());
                return true;
            }
        };
    }

    public Action turretStop() {
        return packet -> {
            turretServo.setPower(0);
            return false;
        };
    }

    public Action turretTimedOutAndBack(double power, double seconds) {
        return new Action() {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean reversing = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    turretServo.setPower(power);
                    timer.reset();
                    initialized = true;
                }

                if (!reversing && timer.seconds() >= seconds) {
                    reversing = true;
                    timer.reset();
                    turretServo.setPower(-power);
                }

                if (reversing && timer.seconds() >= seconds) {
                    turretServo.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    // =========================
    // SEQUENCE (NO SHOOTER CONTROL — RR DOES THAT)
    // - Uses ONLY color4
    // - Only validates first 2 balls (3rd assumed correct)
    // - If UNKNOWN: sample for 0.7s at shoot position
    // - NEVER counts a shot unless it validated (or is 3rd ball)
    // - “No 2-slot jump”: mismatch search is +1 preference
    // - “Backtrack 1”: next shot prefers -1 so it re-tries the skipped one
    // =========================
    public Action runSequence3ShotsNoShooter() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                sequenceRunning = true;

                p.put("Seq/State", seqState.toString());
                p.put("Seq/ShotsDone", shotsDone);
                p.put("Seq/Pattern", desiredPattern.toString());
                p.put("Seq/TargetIdx", targetIdx);
                p.put("Seq/TargetPos", targetPos);

                switch (seqState) {

                    case INIT: {
                        int nearest = nearestShootPos(revolver.getCurrentPosition());
                        currentIdx = idxFromPos(nearest);

                        shotsDone = 0;
                        shotMask[0] = shotMask[1] = shotMask[2] = false;

                        targetIdx = currentIdx;
                        targetPos = posFromIdx(targetIdx);
                        setTargetInternal(targetPos);

                        stateTimer.reset();
                        moveTimer.reset();
                        unknownScanTimer.reset();
                        lastNonUnknownSeen = BallColor.UNKNOWN;

                        seqState = S.MOVE_TO_TARGET;
                        break;
                    }

                    case MOVE_TO_TARGET: {
                        if (atTarget(targetPos) || moveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            stateTimer.reset();
                            unknownScanTimer.reset();
                            lastNonUnknownSeen = BallColor.UNKNOWN;
                            seqState = S.WAIT_VALIDATE;
                        }
                        break;
                    }

                    case WAIT_VALIDATE: {
                        if (stateTimer.seconds() >= VALIDATE_DELAY_SEC) {
                            unknownScanTimer.reset();
                            lastNonUnknownSeen = BallColor.UNKNOWN;
                            seqState = S.READ_VALIDATE;
                        }
                        break;
                    }

                    case READ_VALIDATE: {
                        BallColor expected = expectedForShot(desiredPattern, shotsDone);

                        BallColor seen = readColor4();
                        if (seen != BallColor.UNKNOWN) lastNonUnknownSeen = seen;

                        boolean doColorCheck = !(ONLY_VALIDATE_FIRST_TWO && shotsDone >= 2);

                        p.put("Seq/Expected", expected.toString());
                        p.put("Seq/Seen", seen.toString());
                        p.put("Seq/DoColorCheck", doColorCheck);
                        p.put("Seq/Hue", getHue(color4));

                        // For first 2 shots: require a match (with UNKNOWN scan window)
                        if (doColorCheck) {
                            if (seen == BallColor.UNKNOWN) {
                                if (unknownScanTimer.seconds() < UNKNOWN_SCAN_SEC) {
                                    break; // keep sampling in place
                                }
                                if (lastNonUnknownSeen != BallColor.UNKNOWN) {
                                    seen = lastNonUnknownSeen;
                                }
                            }

                            if (seen != expected) {
                                // mismatch -> advance to next unshot (+1 preference only)
                                int nextIdx = nextUnshotIdxForwardOnly(targetIdx, shotMask);
                                if (nextIdx == -1) {
                                    // nowhere else to try -> just end (don’t “shoot empty” / don’t count)
                                    setTargetInternal(POS_INTAKE);
                                    targetPos = POS_INTAKE;
                                    moveTimer.reset();
                                    seqState = S.RETURN_INTAKE;
                                    break;
                                }
                                targetIdx = nextIdx;
                                targetPos = posFromIdx(targetIdx);
                                setTargetInternal(targetPos);
                                moveTimer.reset();
                                seqState = S.MOVE_TO_TARGET;
                                break;
                            }
                        } else {
                            // 3rd shot: no color check (as requested)
                            // still prevent “UNKNOWN scan” delays
                        }

                        // validated
                        stateTimer.reset();
                        seqState = S.PRE_SHOT_REV_DELAY;
                        break;
                    }

                    case PRE_SHOT_REV_DELAY: {
                        if (stateTimer.seconds() >= PRE_SHOT_REV_DELAY_SEC) {
                            stateTimer.reset();
                            seqState = S.ARM_UP_STATE;
                        }
                        break;
                    }

                    case ARM_UP_STATE: {
                        arm.setPosition(ARM_UP);
                        if (stateTimer.seconds() >= ARM_UPDOWN_SEC) {
                            stateTimer.reset();
                            seqState = S.ARM_DOWN_STATE;
                        }
                        break;
                    }

                    case ARM_DOWN_STATE: {
                        arm.setPosition(ARM_DOWN);
                        if (stateTimer.seconds() >= ARM_UPDOWN_SEC) {
                            stateTimer.reset();
                            seqState = S.POST_SHOT_SETTLE;
                        }
                        break;
                    }

                    case POST_SHOT_SETTLE: {
                        if (stateTimer.seconds() >= BALL_SETTLE_SEC) {
                            seqState = S.NEXT_SHOT;
                        }
                        break;
                    }

                    case NEXT_SHOT: {
                        // Count ONLY when we actually validated/reached shooting states
                        shotMask[targetIdx] = true;
                        shotsDone++;

                        currentIdx = targetIdx;

                        if (shotsDone >= 3) {
                            setTargetInternal(POS_INTAKE);
                            targetPos = POS_INTAKE;
                            moveTimer.reset();
                            seqState = S.RETURN_INTAKE;
                            break;
                        }

                        int nextIdx;
                        if (ONLY_VALIDATE_FIRST_TWO && shotsDone >= 2) {
                            // last ball: go to remaining unshot
                            nextIdx = findOnlyRemainingUnshot(shotMask);
                        } else {
                            // prefer backtrack 1 slot first
                            nextIdx = preferBacktrackOne(currentIdx, shotMask);
                        }

                        if (nextIdx == -1) {
                            setTargetInternal(POS_INTAKE);
                            targetPos = POS_INTAKE;
                            moveTimer.reset();
                            seqState = S.RETURN_INTAKE;
                            break;
                        }

                        targetIdx = nextIdx;
                        targetPos = posFromIdx(targetIdx);
                        setTargetInternal(targetPos);

                        moveTimer.reset();
                        stateTimer.reset();
                        unknownScanTimer.reset();
                        lastNonUnknownSeen = BallColor.UNKNOWN;

                        seqState = S.MOVE_TO_TARGET;
                        break;
                    }

                    case RETURN_INTAKE: {
                        if (atTarget(POS_INTAKE) || moveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            seqState = S.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        arm.setPosition(ARM_DOWN);
                        sequenceRunning = false;
                        resetSequenceState();
                        return false;
                    }
                }

                return true;
            }
        };
    }

    // =========================
    // INTERNAL HELPERS (REV)
    // =========================
    private void setTargetInternal(int pos) {
        targetPosition = clamp(pos, 0, MAX_POSITION);
    }

    private boolean atTarget(int target) {
        return Math.abs(revolver.getCurrentPosition() - target) <= AT_TARGET_TOL;
    }

    private int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private int idxFromPos(int pos) {
        for (int i = 0; i < 3; i++) if (SHOOT_POS[i] == pos) return i;
        return 0;
    }

    private int posFromIdx(int idx) {
        idx = wrap3(idx);
        return SHOOT_POS[idx];
    }

    private int wrap3(int i) {
        i %= 3;
        if (i < 0) i += 3;
        return i;
    }

    private int nearestShootPos(int ticks) {
        int best = POS_SHOOT0;
        int bestErr = Math.abs(ticks - POS_SHOOT0);

        int e1 = Math.abs(ticks - POS_SHOOT1);
        if (e1 < bestErr) { bestErr = e1; best = POS_SHOOT1; }

        int e2 = Math.abs(ticks - POS_SHOOT2);
        if (e2 < bestErr) { bestErr = e2; best = POS_SHOOT2; }

        return best;
    }

    // Forward-only “search next” so it can’t jump 2 slots.
    // Checks +1 then +2, but always chooses +1 if available.
    private int nextUnshotIdxForwardOnly(int fromIdx, boolean[] shot) {
        int a = wrap3(fromIdx + 1);
        if (!shot[a]) return a;

        int b = wrap3(fromIdx + 2);
        if (!shot[b]) return b;

        return -1;
    }

    // Prefer backtrack 1 slot so you re-check the skipped one
    private int preferBacktrackOne(int curIdx, boolean[] shot) {
        int back = wrap3(curIdx - 1);
        if (!shot[back]) return back;

        int cur = wrap3(curIdx);
        if (!shot[cur]) return cur;

        int fwd = wrap3(curIdx + 1);
        if (!shot[fwd]) return fwd;

        return -1;
    }

    private int findOnlyRemainingUnshot(boolean[] shot) {
        int last = -1;
        for (int i = 0; i < 3; i++) if (!shot[i]) last = i;
        return last;
    }

    private void resetSequenceState() {
        seqState = S.INIT;

        shotsDone = 0;
        currentIdx = 0;
        targetIdx = 0;
        targetPos = POS_SHOOT0;

        shotMask[0] = shotMask[1] = shotMask[2] = false;

        stateTimer.reset();
        moveTimer.reset();
        unknownScanTimer.reset();
        lastNonUnknownSeen = BallColor.UNKNOWN;
    }

    // =========================
    // INTERNAL HELPERS (COLOR4)
    // =========================
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

    private BallColor expectedForShot(Pattern p, int shotIdx) {
        if (p == Pattern.GPP) {
            return (shotIdx == 0) ? BallColor.GREEN : BallColor.PURPLE;
        } else if (p == Pattern.PGP) {
            return (shotIdx == 1) ? BallColor.GREEN : BallColor.PURPLE;
        } else { // PPG
            return (shotIdx == 2) ? BallColor.GREEN : BallColor.PURPLE;
        }
    }

    // =========================
    // INTERNAL HELPERS (TURRET)
    // =========================
    private void runTurretLogic() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double power = -calculateTurretPID(tx);

            int pos = turretEncoderMotor.getCurrentPosition();
            if (turretLimits) {
                if (pos >= turretMaxEnc && power > 0) power = 0;
                else if (pos <= turretMinEnc && power < 0) power = 0;
            }

            turretServo.setPower(power);
        } else {
            turretServo.setPower(0);
        }
    }

    private double calculateTurretPID(double error) {
        double dt = turretPidTimer.seconds();
        if (dt <= 0) dt = 0.02;
        turretPidTimer.reset();

        if (Math.abs(error) < turretTolerance) {
            turretLastError = 0;
            return 0;
        }

        double P = turretP * error;
        double D = turretD * (error - turretLastError) / dt;
        turretLastError = error;

        double out = P + D;

        if (Math.abs(out) < turretMinPower) out = Math.signum(out) * turretMinPower;

        if (out > turretMaxPower) out = turretMaxPower;
        if (out < -turretMaxPower) out = -turretMaxPower;

        return out;
    }
}
