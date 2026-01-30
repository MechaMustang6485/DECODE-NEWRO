package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NEWRO.Processors.ColorSensorProcessor;

import java.util.List;

@Disabled
@Config
@TeleOp(name = "TestSequenceStackKnownGood", group = "TeleOp")
public class TestSequenceStackKnownGood extends OpMode {

    // =========================
    // DRIVE
    // =========================
    private DcMotor fl, fr, bl, br;
    private IMU imu;
    public static double DRIVE_POWER = 0.8;

    // =========================
    // LIMELIGHT
    // =========================
    private Limelight3A limelight;
    public static int LIMELIGHT_PIPELINE = 9;
    private int lastSequenceId = -1;

    // =========================
    // LIGHTS
    // =========================
    private RevBlinkinLedDriver lights;
    public static RevBlinkinLedDriver.BlinkinPattern LED_OK = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public static RevBlinkinLedDriver.BlinkinPattern LED_BAD = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static RevBlinkinLedDriver.BlinkinPattern LED_IDLE = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    public static RevBlinkinLedDriver.BlinkinPattern LED_RUNNING = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    // =========================
    // INTAKE + TOUCH
    // =========================
    private DcMotorEx intake;
    private TouchSensor touch;
    public static double TOUCH_DELAY_SEC = 1.0;
    public static int MAX_BALLS = 3;

    private boolean touchPrev = false;
    private boolean waitingSample = false;
    private final ElapsedTime touchTimer = new ElapsedTime();

    private int ballsLoaded = 0;
    private boolean touchEnabled = true;

    // which pocket is currently aligned to intake during loading (0..2)
    private int loadIndex = 0;

    // =========================
    // REVOLVER
    // =========================
    private DcMotorEx revolver;
    private PIDController revPID;

    public static double revP = 0.1, revI = 0, revD = 0.0002;

    public static int REV_HOME = 0;     // intake alignment
    public static int REV_SLOT = 96;    // one pocket step
    public static int REV_SHOOT = 48;   // shoot alignment base

    public static int revTarget = REV_HOME;

    public static double REV_DEADBAND = 5;
    public static double REV_MIN_POWER = 0.06;
    public static double REV_MAX_POWER = 0.6;

    // =========================
    // SHOOTER
    // =========================
    private DcMotorEx shooterT, shooterB;
    public static double pshot = 7.3013, ishot = 0, dshot = 0, fshot = 10;
    public static double SHOOT_VEL = 5000;

    // =========================
    // ARM
    // =========================
    private Servo arm;
    public static double ARM_DOWN = 0.0;
    public static double ARM_UP = 0.3;

    // =========================
    // COLOR
    // color1 (intake) = quick scan
    // color4 (shooter) = final say
    // =========================
    private ColorSensorProcessor colorProc;      // gives us color1/2/3 (we only use color1)
    private NormalizedColorSensor color4;        // shooter sensor

    public enum BallSlot { EMPTY, FULL, PURPLE, GREEN }

    // memory snapshot for the 3 pockets
    private BallSlot[] slotMemory = { BallSlot.EMPTY, BallSlot.EMPTY, BallSlot.EMPTY };

    // =========================
    // PATTERN
    // =========================
    enum Pattern { GPP, PGP, PPG }
    private Pattern desiredPattern = Pattern.GPP;

    // =========================
    // AUTO FSM
    // =========================
    enum AutoState {
        IDLE,

        // verify all pockets at intake using color1
        VERIFY_MOVE,
        VERIFY_WAIT_1S,
        VERIFY_READ,
        VERIFY_ADVANCE,

        PLAN,

        // per-shot:
        MOVE_TO_SHOOT,
        SHOOT_VERIFY_WAIT,
        SHOOT_VERIFY_READ,   // uses color4
        SPINUP,
        ARM_UP_STATE,
        ARM_DOWN_STATE,
        NEXT_SHOT,

        DONE,
        ABORT
    }

    private AutoState autoState = AutoState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean shootRequested = false;

    // verify scan index
    private int verifyIndex = 0;

    // plan order (slot indices 0..2)
    private int[] plan = null;
    private int shotIdx = 0;

    // per-shot verify attempts (if wrong color at color4, index and retry)
    private int verifyTriesThisShot = 0;
    public static int MAX_VERIFY_TRIES = 3;

    // timings
    public static double VERIFY_MOVE_SETTLE_SEC = 0.20;
    public static double VERIFY_WAIT_SEC = 1.0;

    public static double MOVE_SETTLE_SEC = 0.20;
    public static double SHOOT_VERIFY_WAIT_SEC = 0.20;

    public static double SPINUP_SEC = 0.80;
    public static double ARM_HOLD_SEC = 0.35;
    public static double ARM_DOWN_SEC = 0.20;

    // =========================
    // INIT / LOOP
    // =========================
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drive
        fl = hardwareMap.get(DcMotor.class, "Fl");
        fr = hardwareMap.get(DcMotor.class, "Fr");
        bl = hardwareMap.get(DcMotor.class, "Bl");
        br = hardwareMap.get(DcMotor.class, "Br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.setPollRateHz(100);

        // Lights
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(LED_IDLE);

        // Intake + Touch
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touch = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Revolver
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        revPID = new PIDController(revP, revI, revD);
        revTarget = REV_HOME;

        // Shooter
        shooterT = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Arm
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(ARM_DOWN);

        // Colors
        colorProc = new ColorSensorProcessor(hardwareMap);
        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        color4.setGain(1);

        resetAll();
    }

    @Override
    public void start() {
        limelight.start();
        timer.reset();
    }

    @Override
    public void stop() {
        limelight.stop();
        shooterT.setVelocity(0);
        shooterB.setVelocity(0);
        intake.setPower(0);
        revolver.setPower(0);
        arm.setPosition(ARM_DOWN);
        lights.setPattern(LED_IDLE);
    }

    @Override
    public void loop() {
        // Intake
        if (gamepad1.dpad_left) intake.setPower(1);
        else if (gamepad1.dpad_right) intake.setPower(0);

        // Drive
        driveFieldCentric();

        // Limelight -> desiredPattern
        updatePatternFromLimelight();

        // Revolver PID always
        runRevolverPID();

        // Shooter PID always
        applyShooterPID();

        // Loading (touch) only when not auto
        handleTouchLoad();

        // Start button
        if (autoState == AutoState.IDLE) {
            if (gamepad1.yWasPressed() && isFull()) {
                shootRequested = true;
            }
        } else {
            if (gamepad1.bWasPressed()) {
                autoState = AutoState.ABORT;
            }
        }

        // Auto
        runAuto();

        // Lights
        updateLights();

        // Telemetry
        telemetryOut();
    }

    // =========================
    // LOADING: touch -> wait 1s -> read color1 -> store -> advance
    // =========================
    private void handleTouchLoad() {
        if (!touchEnabled) return;
        if (autoState != AutoState.IDLE) return;

        boolean pressed = touch.isPressed();

        if (pressed && !touchPrev && !waitingSample) {
            waitingSample = true;
            touchTimer.reset();
        }

        if (waitingSample && touchTimer.seconds() >= TOUCH_DELAY_SEC) {
            waitingSample = false;

            // align current pocket to intake
            revTarget = REV_HOME + (loadIndex * REV_SLOT);

            BallSlot c = readBallSlotFromColor1();
            slotMemory[loadIndex] = c;

            ballsLoaded++;
            loadIndex++;

            if (ballsLoaded >= MAX_BALLS) {
                touchEnabled = false;
            } else {
                // align next pocket to intake
                revTarget = REV_HOME + (loadIndex * REV_SLOT);
            }
        }

        touchPrev = pressed;
    }

    // =========================
    // AUTO: verify all pockets at intake (color1), then plan, then shoot using color4 final-say
    // =========================
    private void runAuto() {
        switch (autoState) {
            case IDLE: {
                if (!shootRequested) break;
                shootRequested = false;

                // start verify cycle
                verifyIndex = 0;
                revTarget = REV_HOME + (verifyIndex * REV_SLOT);
                timer.reset();
                autoState = AutoState.VERIFY_MOVE;
                break;
            }

            case VERIFY_MOVE: {
                if (timer.seconds() >= VERIFY_MOVE_SETTLE_SEC) {
                    timer.reset();
                    autoState = AutoState.VERIFY_WAIT_1S;
                }
                break;
            }

            case VERIFY_WAIT_1S: {
                if (timer.seconds() >= VERIFY_WAIT_SEC) {
                    timer.reset();
                    autoState = AutoState.VERIFY_READ;
                }
                break;
            }

            case VERIFY_READ: {
                // quick scan at intake using color1
                slotMemory[verifyIndex] = readBallSlotFromColor1();
                timer.reset();
                autoState = AutoState.VERIFY_ADVANCE;
                break;
            }

            case VERIFY_ADVANCE: {
                verifyIndex++;
                if (verifyIndex >= 3) {
                    autoState = AutoState.PLAN;
                } else {
                    revTarget = REV_HOME + (verifyIndex * REV_SLOT);
                    timer.reset();
                    autoState = AutoState.VERIFY_MOVE;
                }
                break;
            }

            case PLAN: {
                plan = planFromMemoryOrFallback();
                shotIdx = 0;
                verifyTriesThisShot = 0;

                // move to first shot
                revTarget = ticksForShootSlot(plan[shotIdx]);
                timer.reset();
                autoState = AutoState.MOVE_TO_SHOOT;
                break;
            }

            case MOVE_TO_SHOOT: {
                if (timer.seconds() >= MOVE_SETTLE_SEC) {
                    timer.reset();
                    autoState = AutoState.SHOOT_VERIFY_WAIT;
                }
                break;
            }

            case SHOOT_VERIFY_WAIT: {
                if (timer.seconds() >= SHOOT_VERIFY_WAIT_SEC) {
                    timer.reset();
                    autoState = AutoState.SHOOT_VERIFY_READ;
                }
                break;
            }

            case SHOOT_VERIFY_READ: {
                // FINAL SAY sensor at shooter
                BallSlot seen = readBallSlotFromColor4();
                BallSlot expected = expectedForShotIndex(desiredPattern, shotIdx);

                boolean sequencePossible = canDoSequenceFromMemory(slotMemory);

                // If we can't do a real sequence anyway, just shoot (no searching)
                if (!sequencePossible) {
                    startSpinup();
                    break;
                }

                // If color4 is UNKNOWN, don't stall forever: try a couple, then shoot anyway
                if (seen == BallSlot.FULL || seen == BallSlot.EMPTY) {
                    if (verifyTriesThisShot < MAX_VERIFY_TRIES) {
                        verifyTriesThisShot++;
                        // index 1 slot and try again
                        revTarget += REV_SLOT;
                        timer.reset();
                        autoState = AutoState.MOVE_TO_SHOOT;
                    } else {
                        startSpinup();
                    }
                    break;
                }

                // If it matches expected, shoot
                if (seen == expected) {
                    startSpinup();
                } else {
                    // mismatch: try to find the right one by indexing
                    if (verifyTriesThisShot < MAX_VERIFY_TRIES) {
                        verifyTriesThisShot++;
                        revTarget += REV_SLOT;
                        timer.reset();
                        autoState = AutoState.MOVE_TO_SHOOT;
                    } else {
                        // give up and shoot anyway
                        startSpinup();
                    }
                }
                break;
            }

            case SPINUP: {
                if (timer.seconds() >= SPINUP_SEC) {
                    arm.setPosition(ARM_UP);
                    timer.reset();
                    autoState = AutoState.ARM_UP_STATE;
                }
                break;
            }

            case ARM_UP_STATE: {
                if (timer.seconds() >= ARM_HOLD_SEC) {
                    arm.setPosition(ARM_DOWN);
                    timer.reset();
                    autoState = AutoState.ARM_DOWN_STATE;
                }
                break;
            }

            case ARM_DOWN_STATE: {
                if (timer.seconds() >= ARM_DOWN_SEC) {
                    autoState = AutoState.NEXT_SHOT;
                }
                break;
            }

            case NEXT_SHOT: {
                shotIdx++;
                verifyTriesThisShot = 0;

                if (shotIdx >= 3) {
                    autoState = AutoState.DONE;
                } else {
                    revTarget = ticksForShootSlot(plan[shotIdx]);
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                }
                break;
            }

            case DONE: {
                shooterT.setVelocity(0);
                shooterB.setVelocity(0);
                arm.setPosition(ARM_DOWN);

                // return to intake
                revTarget = REV_HOME;
                resetAll();
                autoState = AutoState.IDLE;
                break;
            }

            case ABORT: {
                shooterT.setVelocity(0);
                shooterB.setVelocity(0);
                arm.setPosition(ARM_DOWN);

                revTarget = REV_HOME;
                resetAll();
                autoState = AutoState.IDLE;
                break;
            }
        }
    }

    private void startSpinup() {
        shooterT.setVelocity(SHOOT_VEL);
        shooterB.setVelocity(SHOOT_VEL);
        timer.reset();
        autoState = AutoState.SPINUP;
    }

    // =========================
    // Planning
    // =========================
    private int[] planFromMemoryOrFallback() {
        // If exactly 1 G + 2 P, we can attempt sequence planning
        if (!canDoSequenceFromMemory(slotMemory)) {
            return new int[]{0, 1, 2};
        }

        BallSlot[] expect = expectedPattern(desiredPattern);
        boolean[] used = new boolean[]{false, false, false};
        int[] order = new int[3];

        for (int i = 0; i < 3; i++) {
            int idx = findFirstMatch(slotMemory, expect[i], used);
            if (idx == -1) return new int[]{0, 1, 2};
            order[i] = idx;
            used[idx] = true;
        }

        return order;
    }

    private boolean canDoSequenceFromMemory(BallSlot[] mem) {
        int g = 0, p = 0;
        for (BallSlot s : mem) {
            if (s == BallSlot.GREEN) g++;
            if (s == BallSlot.PURPLE) p++;
        }
        return (g == 1 && p == 2);
    }

    private int findFirstMatch(BallSlot[] mem, BallSlot want, boolean[] used) {
        for (int i = 0; i < 3; i++) {
            if (!used[i] && mem[i] == want) return i;
        }
        return -1;
    }

    private BallSlot[] expectedPattern(Pattern p) {
        if (p == Pattern.GPP) return new BallSlot[]{BallSlot.GREEN, BallSlot.PURPLE, BallSlot.PURPLE};
        if (p == Pattern.PGP) return new BallSlot[]{BallSlot.PURPLE, BallSlot.GREEN, BallSlot.PURPLE};
        return new BallSlot[]{BallSlot.PURPLE, BallSlot.PURPLE, BallSlot.GREEN};
    }

    // expected color “by shot index” (0..2) for the pattern
    private BallSlot expectedForShotIndex(Pattern p, int shotIndex) {
        BallSlot[] e = expectedPattern(p);
        return e[Math.max(0, Math.min(2, shotIndex))];
    }

    private int ticksForShootSlot(int slotIdx) {
        return REV_SHOOT + (REV_SLOT * slotIdx);
    }

    // =========================
    // REVOLVER PID
    // =========================
    private void runRevolverPID() {
        int pos = revolver.getCurrentPosition();
        int err = revTarget - pos;

        if (Math.abs(err) <= REV_DEADBAND) {
            revolver.setPower(0);
            return;
        }

        double pwr = revPID.calculate(pos, revTarget);

        if (Math.abs(pwr) < REV_MIN_POWER) {
            pwr = Math.signum(err) * REV_MIN_POWER;
        }

        pwr = Math.max(-REV_MAX_POWER, Math.min(REV_MAX_POWER, pwr));
        revolver.setPower(pwr);
    }

    // =========================
    // SHOOTER PIDF
    // =========================
    private void applyShooterPID() {
        PIDFCoefficients c = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }

    // =========================
    // LIMELIGHT -> desired pattern (21=GPP,22=PGP,23=PPG)
    // =========================
    private void updatePatternFromLimelight() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) {
            lastSequenceId = -1;
            return;
        }

        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            lastSequenceId = -1;
            return;
        }

        int bestId = -1;
        double bestArea = -1;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id != 21 && id != 22 && id != 23) continue;
            double area = t.getTargetArea();
            if (area > bestArea) {
                bestArea = area;
                bestId = id;
            }
        }

        if (bestId == -1) {
            lastSequenceId = -1;
            return;
        }

        lastSequenceId = bestId;
        if (bestId == 21) desiredPattern = Pattern.GPP;
        else if (bestId == 22) desiredPattern = Pattern.PGP;
        else desiredPattern = Pattern.PPG;
    }

    // =========================
    // COLOR READERS
    // =========================
    private BallSlot readBallSlotFromColor1() {
        // use your processor classification for color1
        ColorSensorProcessor.BallSlot s = colorProc.colorInSlot1();
        if (s == ColorSensorProcessor.BallSlot.GREEN) return BallSlot.GREEN;
        if (s == ColorSensorProcessor.BallSlot.PURPLE) return BallSlot.PURPLE;
        return BallSlot.FULL;
    }

    private BallSlot readBallSlotFromColor4() {
        float hue = getHue(color4);
        // same thresholds you used:
        if (hue >= 100 && hue <= 180) return BallSlot.GREEN;
        if (hue >= 181 && hue <= 255) return BallSlot.PURPLE;
        return BallSlot.FULL; // unknown
    }

    private float getHue(NormalizedColorSensor sensor) {
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    // =========================
    // FULL
    // =========================
    private boolean isFull() {
        return ballsLoaded >= MAX_BALLS;
    }

    // =========================
    // RESET ALL (clears memory and re-enables loading)
    // =========================
    private void resetAll() {
        slotMemory[0] = BallSlot.EMPTY;
        slotMemory[1] = BallSlot.EMPTY;
        slotMemory[2] = BallSlot.EMPTY;

        ballsLoaded = 0;
        touchEnabled = true;
        waitingSample = false;
        touchPrev = false;

        loadIndex = 0;
        shootRequested = false;

        plan = null;
        shotIdx = 0;
        verifyIndex = 0;
        verifyTriesThisShot = 0;
    }

    // =========================
    // LIGHTS
    // =========================
    private void updateLights() {
        if (autoState != AutoState.IDLE) {
            lights.setPattern(LED_RUNNING);
            return;
        }

        if (!isFull()) {
            lights.setPattern(LED_IDLE);
            return;
        }

        // green only if we can do a sequence (memory says 1G2P)
        boolean canSeq = canDoSequenceFromMemory(slotMemory);
        lights.setPattern(canSeq ? LED_OK : LED_BAD);
    }

    // =========================
    // DRIVE (field-centric)
    // =========================
    private void driveFieldCentric() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double r = gamepad1.right_stick_x;

        double h = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rx = x * Math.cos(-h) - y * Math.sin(-h);
        double ry = x * Math.sin(-h) + y * Math.cos(-h);

        double d = Math.max(Math.abs(rx) + Math.abs(ry) + Math.abs(r), 1);

        fl.setPower((ry + rx + r) / d * DRIVE_POWER);
        bl.setPower((ry - rx + r) / d * DRIVE_POWER);
        fr.setPower((ry - rx - r) / d * DRIVE_POWER);
        br.setPower((ry + rx - r) / d * DRIVE_POWER);

        if (gamepad1.back) imu.resetYaw();
    }

    // =========================
    // TELEMETRY
    // =========================
    private void telemetryOut() {
        telemetry.addData("Sequence Btn", "Y (only when 3 balls)");
        telemetry.addData("AutoState", autoState);

        telemetry.addData("Limelight ID", lastSequenceId);
        telemetry.addData("DesiredPattern", desiredPattern);

        telemetry.addData("BallsLoaded", ballsLoaded);
        telemetry.addData("TouchEnabled", touchEnabled);
        telemetry.addData("LoadIndex", loadIndex);

        telemetry.addData("Mem0", slotMemory[0]);
        telemetry.addData("Mem1", slotMemory[1]);
        telemetry.addData("Mem2", slotMemory[2]);

        telemetry.addData("RevPos", revolver.getCurrentPosition());
        telemetry.addData("RevTarget", revTarget);

        telemetry.addData("Color1", colorProc.colorInSlot1());
        telemetry.addData("Color4(hue)", "%.1f", getHue(color4));
        telemetry.addData("Color4(class)", readBallSlotFromColor4());

        telemetry.addData("ShotIdx", shotIdx);
        telemetry.addData("VerifyTries", verifyTriesThisShot);

        telemetry.update();
    }
}
