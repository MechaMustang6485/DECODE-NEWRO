package org.firstinspires.ftc.teamcode.NewRo2.Testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Disabled
@Config
@TeleOp(name = "TestSequenceStack_Color4Only_DumpFallback_final_final_ultimate_final", group = "TeleOp")
public class TestSequenceStack extends OpMode {

    // =========================
    // DRIVE (PINPOINT ONLY)
    // =========================
    private DcMotor fl, fr, bl, br;
    private GoBildaPinpointDriver pinpoint;
    public static double DRIVE_POWER = 0.8;

    public static double PINPOINT_OFFSET_X_MM = 76.2;
    public static double PINPOINT_OFFSET_Y_MM = 127.0;

    // =========================
    // IMU (Limelight orientation update)
    // =========================
    private IMU imu;

    // =========================
    // LIMELIGHT (pattern tag 21/22/23)
    // =========================
    private Limelight3A limelight;
    public static int LIMELIGHT_PIPELINE = 0;
    private int lastSequenceId = -1;

    // =========================
    // LIGHTS
    // =========================
    private RevBlinkinLedDriver lights;
    public static RevBlinkinLedDriver.BlinkinPattern LED_OK = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public static RevBlinkinLedDriver.BlinkinPattern LED_IDLE = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    public static RevBlinkinLedDriver.BlinkinPattern LED_RUNNING = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public static RevBlinkinLedDriver.BlinkinPattern LED_ABORT = RevBlinkinLedDriver.BlinkinPattern.RED;

    // =========================
    // INTAKE + TOUCH (COUNT ONLY)
    // =========================
    private DcMotorEx intake;
    private TouchSensor touch;

    public static int MAX_BALLS = 3;
    private int ballsLoaded = 0;
    private boolean touchEnabled = true;
    private boolean touchPrev = false;
    private int loadIndex = 0;

    // =========================
    // REVOLVER
    // =========================
    private DcMotorEx revolver;
    private PIDController revPID;

    public static double revP = 0.1, revI = 0, revD = 0.0002;

    public static int REV_HOME = 0;     // intake alignment base
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
    // COLOR4 ONLY (FINAL SAY)
    // =========================
    private NormalizedColorSensor color4;
    public enum BallSlot { UNKNOWN, PURPLE, GREEN }

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

        START_SPINUP,
        SPINUP_WAIT,

        MOVE_TO_SHOOT,
        MOVE_SETTLE,

        VERIFY_WAIT,
        VERIFY_READ,

        FIRE_ARM_UP,
        FIRE_ARM_DOWN,

        NEXT_SHOT,
        DONE,
        ABORT
    }

    private AutoState autoState = AutoState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean shootRequested = false;

    private int shotIdx = 0;
    private int scansThisShot = 0;

    private int curShootSlotIdx = 0;

    private final boolean[] firedSlot = new boolean[]{false, false, false};

    // ✅ NEW: dump-all fallback mode
    private boolean dumpAllMode = false;

    // timings you asked for
    public static double INITIAL_SPINUP_SEC = 0.6;        // was 1.0
    public static double MOVE_SETTLE_SEC = 0.12;          // was 0.20
    public static double PER_BALL_VERIFY_WAIT_SEC = 0.4;  // ✅ validation delay (was 0.70)

    // arm timing
    public static double ARM_HOLD_SEC = 0.2;              // ✅ was 0.35
    public static double ARM_DOWN_SEC = 0.2;              // ✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅ was 0.20 already, keep at 0.2


    // scan behavior
    public static int MAX_SCANS_PER_SHOT = 3;
    private static final int DIR_FORWARD = +1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.setPollRateHz(100);

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(LED_IDLE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touch = hardwareMap.get(TouchSensor.class, "touchSensor");

        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revPID = new PIDController(revP, revI, revD);
        revTarget = REV_HOME;

        shooterT = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(ARM_DOWN);

        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        color4.setGain(12);

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

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        lights.setPattern(LED_IDLE);
    }

    @Override
    public void loop() {
        pinpoint.update();

        if (gamepad1.back) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            imu.resetYaw();
        }

        if (gamepad1.dpad_left) intake.setPower(1);
        else if (gamepad1.dpad_right) intake.setPower(0);

        driveFieldCentric();
        updatePatternFromLimelight();

        runRevolverPID();
        applyShooterPID();

        handleTouchLoadCountOnly();

        if (autoState == AutoState.IDLE) {
            if (gamepad1.yWasPressed() && isFull()) {
                shootRequested = true;
            }
        } else {
            if (gamepad1.bWasPressed()) {
                autoState = AutoState.ABORT;
            }
        }

        runAuto();
        updateLights();
        telemetryOut();
    }

    private void handleTouchLoadCountOnly() {
        if (!touchEnabled) return;
        if (autoState != AutoState.IDLE) return;

        boolean pressed = touch.isPressed();

        if (pressed && !touchPrev) {
            ballsLoaded++;
            loadIndex++;

            if (ballsLoaded >= MAX_BALLS) {
                ballsLoaded = MAX_BALLS;
                touchEnabled = false;
            }

            int idx = Math.min(loadIndex, 2);
            revTarget = REV_HOME + (idx * REV_SLOT);
        }

        touchPrev = pressed;
    }

    private void runAuto() {
        switch (autoState) {
            case IDLE: {
                if (!shootRequested) break;
                shootRequested = false;

                shotIdx = 0;
                scansThisShot = 0;
                dumpAllMode = false;

                firedSlot[0] = false;
                firedSlot[1] = false;
                firedSlot[2] = false;

                curShootSlotIdx = 0;
                setShootSlot(curShootSlotIdx);

                autoState = AutoState.START_SPINUP;
                timer.reset();
                break;
            }

            case START_SPINUP: {
                shooterT.setVelocity(SHOOT_VEL);
                shooterB.setVelocity(SHOOT_VEL);
                timer.reset();
                autoState = AutoState.SPINUP_WAIT;
                break;
            }

            case SPINUP_WAIT: {
                if (timer.seconds() >= INITIAL_SPINUP_SEC) {
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                }
                break;
            }

            case MOVE_TO_SHOOT: {
                timer.reset();
                autoState = AutoState.MOVE_SETTLE;
                break;
            }

            case MOVE_SETTLE: {
                if (timer.seconds() >= MOVE_SETTLE_SEC) {
                    timer.reset();
                    autoState = AutoState.VERIFY_WAIT;
                }
                break;
            }

            case VERIFY_WAIT: {
                if (timer.seconds() >= PER_BALL_VERIFY_WAIT_SEC) {
                    autoState = AutoState.VERIFY_READ;
                }
                break;
            }

            case VERIFY_READ: {
                // If dump-all is active, ignore colors and just shoot remaining slots
                if (dumpAllMode) {
                    // move to next unfired slot if current already fired
                    if (firedSlot[curShootSlotIdx]) {
                        if (!gotoNextUnfiredInOrder()) {
                            autoState = AutoState.DONE;
                            break;
                        }
                        timer.reset();
                        autoState = AutoState.MOVE_TO_SHOOT;
                        break;
                    }

                    // shoot this slot
                    arm.setPosition(ARM_UP);
                    timer.reset();
                    autoState = AutoState.FIRE_ARM_UP;
                    break;
                }

                // pattern mode:
                if (firedSlot[curShootSlotIdx]) {
                    if (!stepToNextUnfired(DIR_FORWARD)) {
                        // nothing left -> done
                        autoState = AutoState.DONE;
                        break;
                    }
                    scansThisShot++;
                    if (scansThisShot > MAX_SCANS_PER_SHOT) {
                        dumpAllMode = true; // ✅ fallback
                    }
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                    break;
                }

                BallSlot seen = readBallSlotFromColor4();
                BallSlot expected = expectedForShotIndex(desiredPattern, shotIdx);

                // not confident -> scan
                if (seen == BallSlot.UNKNOWN) {
                    scansThisShot++;
                    if (scansThisShot > MAX_SCANS_PER_SHOT) {
                        dumpAllMode = true; // ✅ fallback: just shoot all remaining
                        timer.reset();
                        autoState = AutoState.MOVE_TO_SHOOT;
                        break;
                    }

                    stepToNextUnfired(DIR_FORWARD);
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                    break;
                }

                if (seen == expected) {
                    arm.setPosition(ARM_UP);
                    timer.reset();
                    autoState = AutoState.FIRE_ARM_UP;
                } else {
                    scansThisShot++;
                    if (scansThisShot > MAX_SCANS_PER_SHOT) {
                        dumpAllMode = true; // ✅ fallback
                        timer.reset();
                        autoState = AutoState.MOVE_TO_SHOOT;
                        break;
                    }

                    stepToNextUnfired(DIR_FORWARD);
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                }
                break;
            }

            case FIRE_ARM_UP: {
                if (timer.seconds() >= ARM_HOLD_SEC) {
                    arm.setPosition(ARM_DOWN);
                    timer.reset();
                    autoState = AutoState.FIRE_ARM_DOWN;
                }
                break;
            }

            case FIRE_ARM_DOWN: {
                if (timer.seconds() >= ARM_DOWN_SEC) {
                    autoState = AutoState.NEXT_SHOT;
                }
                break;
            }

            case NEXT_SHOT: {
                firedSlot[curShootSlotIdx] = true;

                if (!dumpAllMode) {
                    shotIdx++;
                    scansThisShot = 0;

                    if (shotIdx >= 3) {
                        autoState = AutoState.DONE;
                        break;
                    }

                    // backtrack one slot before searching next expected ball
                    curShootSlotIdx = wrapIdx(curShootSlotIdx - 1);
                    setShootSlot(curShootSlotIdx);
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                } else {
                    // dump-all mode: do NOT backtrack, just go in order 0->1->2
                    if (!gotoNextUnfiredInOrder()) {
                        autoState = AutoState.DONE;
                        break;
                    }
                    timer.reset();
                    autoState = AutoState.MOVE_TO_SHOOT;
                }
                break;
            }

            case DONE: {
                shooterT.setVelocity(0);
                shooterB.setVelocity(0);
                arm.setPosition(ARM_DOWN);

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

    // In dump mode, pick next unfired slot in 0->1->2 order
    private boolean gotoNextUnfiredInOrder() {
        for (int i = 0; i < 3; i++) {
            if (!firedSlot[i]) {
                curShootSlotIdx = i;
                setShootSlot(curShootSlotIdx);
                return true;
            }
        }
        return false;
    }

    private boolean stepToNextUnfired(int dir) {
        for (int k = 0; k < 3; k++) {
            curShootSlotIdx = wrapIdx(curShootSlotIdx + dir);
            if (!firedSlot[curShootSlotIdx]) {
                setShootSlot(curShootSlotIdx);
                return true;
            }
        }
        return false;
    }

    private int wrapIdx(int i) {
        int r = i % 3;
        if (r < 0) r += 3;
        return r;
    }

    private void setShootSlot(int slotIdx) {
        revTarget = ticksForShootSlot(slotIdx);
    }

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

    private void applyShooterPID() {
        PIDFCoefficients c = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }

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

    private BallSlot readBallSlotFromColor4() {
        float hue = getHue(color4);

        if (hue >= 100 && hue <= 180) return BallSlot.GREEN;
        if (hue >= 181 && hue <= 255) return BallSlot.PURPLE;

        return BallSlot.UNKNOWN;
    }

    private float getHue(NormalizedColorSensor sensor) {
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    private BallSlot expectedForShotIndex(Pattern p, int shotIndex) {
        if (p == Pattern.GPP) return (shotIndex == 0) ? BallSlot.GREEN : BallSlot.PURPLE;
        if (p == Pattern.PGP) return (shotIndex == 1) ? BallSlot.GREEN : BallSlot.PURPLE;
        return (shotIndex == 2) ? BallSlot.GREEN : BallSlot.PURPLE;
    }

    private int ticksForShootSlot(int slotIdx) {
        return REV_SHOOT + (REV_SLOT * slotIdx);
    }

    private boolean isFull() {
        return ballsLoaded >= MAX_BALLS;
    }

    private void resetAll() {
        ballsLoaded = 0;
        touchEnabled = true;
        touchPrev = false;
        loadIndex = 0;

        shootRequested = false;
        shotIdx = 0;
        scansThisShot = 0;
        dumpAllMode = false;

        firedSlot[0] = false;
        firedSlot[1] = false;
        firedSlot[2] = false;

        curShootSlotIdx = 0;
    }

    private void updateLights() {
        if (autoState != AutoState.IDLE) {
            lights.setPattern(LED_RUNNING);
            return;
        }

        if (!isFull()) {
            lights.setPattern(LED_IDLE);
            return;
        }

        lights.setPattern(LED_OK);
    }

    private void driveFieldCentric() {
        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double y_input = -gamepad1.left_stick_y;
        double x_input = gamepad1.left_stick_x * 1.1;
        double rotation_input = gamepad1.right_stick_x;

        double rotX = x_input * Math.cos(-botHeading) - y_input * Math.sin(-botHeading);
        double rotY = x_input * Math.sin(-botHeading) + y_input * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation_input), 1);

        double frontLeftPower = (rotY + rotX + rotation_input) / denominator;
        double backLeftPower = (rotY - rotX + rotation_input) / denominator;
        double frontRightPower = (rotY - rotX - rotation_input) / denominator;
        double backRightPower = (rotY + rotX - rotation_input) / denominator;

        fl.setPower(frontLeftPower * DRIVE_POWER);
        bl.setPower(backLeftPower * DRIVE_POWER);
        fr.setPower(frontRightPower * DRIVE_POWER);
        br.setPower(backRightPower * DRIVE_POWER);
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(PINPOINT_OFFSET_X_MM, PINPOINT_OFFSET_Y_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    private void telemetryOut() {
        Pose2D pose = pinpoint.getPosition();

        telemetry.addData("Sequence Btn", "Y (only when 3 balls)");
        telemetry.addData("AutoState", autoState);

        telemetry.addData("Pinpoint X (IN)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y (IN)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading (DEG)", pose.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Limelight ID", lastSequenceId);
        telemetry.addData("DesiredPattern", desiredPattern);

        telemetry.addData("BallsLoaded", ballsLoaded);
        telemetry.addData("TouchEnabled", touchEnabled);

        telemetry.addData("RevPos", revolver.getCurrentPosition());
        telemetry.addData("RevTarget", revTarget);

        telemetry.addData("Color4 hue", "%.1f", getHue(color4));
        telemetry.addData("Color4 class", readBallSlotFromColor4());

        telemetry.addData("ShotIdx", shotIdx);
        telemetry.addData("CurShootSlotIdx", curShootSlotIdx);
        telemetry.addData("ScansThisShot", scansThisShot);
        telemetry.addData("DumpAllMode", dumpAllMode);

        telemetry.addData("Fired0", firedSlot[0]);
        telemetry.addData("Fired1", firedSlot[1]);
        telemetry.addData("Fired2", firedSlot[2]);

        telemetry.update();
    }
}
