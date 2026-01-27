package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp
public class opendv extends OpMode {

    // Drive
    private DcMotorEx fl, fr, br, bl;
    private IMU imu;
    private double headingOffsetRad = 0.0;

    // Turret (CRServo) + encoder source (motorOne)
    private CRServo turret;
    private DcMotorEx motorOne; // encoder that measures turret rotation (external encoder or mechanically linked)\

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ===== Dashboard tunables =====
    public static double DRIVE_DEADBAND = 0.05;
    public static double DRIVE_TURN_SCALE = 1.0;
    public static double DRIVE_SPEED_SCALE = 1.0;

    // Tracking
    public static double kP = 0.0075;          // power per degree of bearing
    public static double deadband = 3.5;     // degrees
    public static int TARGET_ID = 24;        // preferred tag id
    public static int LIMIT_LEFT = 100;     // encoder ticks (set these!)
    public static int LIMIT_RIGHT = -100;   // encoder ticks (set these!)

    public static double TURRET_MAX_POWER = 0.7;   // clamp
    public static double SEARCH_POWER = 0.25;      // turret scan power
    public static int SOFT_ZONE_TICKS = 150;       // slows near ends (optional)

    // Search state
    private boolean searchingLeft = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drive motors
        fl = hardwareMap.get(DcMotorEx.class, "Fl");
        fr = hardwareMap.get(DcMotorEx.class, "Fr");
        bl = hardwareMap.get(DcMotorEx.class, "Bl");
        br = hardwareMap.get(DcMotorEx.class, "Br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret servo
        turret = hardwareMap.crservo.get("t");

        // Encoder source (YOU said: encoder is on motorOne)
        motorOne = hardwareMap.get(DcMotorEx.class, "motorOne");
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOne.setPower(0); // encoder-only (keep at 0 so it doesn't fight anything)

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        headingOffsetRad = 0.0;

        // AprilTag
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camcam"))
                .addProcessor(aprilTag)
                .build();
    }

    // =========================
    // Heading / drive helpers
    // =========================
    private double getHeadingRad() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double yawRad = ypr.getYaw(AngleUnit.RADIANS);
        return yawRad - headingOffsetRad;
    }

    private static double applyDeadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y = applyDeadband(y, DRIVE_DEADBAND);
        x = applyDeadband(x, DRIVE_DEADBAND);
        rx = applyDeadband(rx, DRIVE_DEADBAND);

        rx *= DRIVE_TURN_SCALE;

        double heading = getHeadingRad();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double rotX = x * cos - y * sin;
        double rotY = x * sin + y * cos;

        double flp = rotY + rotX + rx;
        double frp = rotY - rotX - rx;
        double blp = rotY - rotX + rx;
        double brp = rotY + rotX - rx;

        double max = Math.max(1.0, Math.max(Math.abs(flp),
                Math.max(Math.abs(frp), Math.max(Math.abs(blp), Math.abs(brp)))));

        flp = (flp / max) * DRIVE_SPEED_SCALE;
        frp = (frp / max) * DRIVE_SPEED_SCALE;
        blp = (blp / max) * DRIVE_SPEED_SCALE;
        brp = (brp / max) * DRIVE_SPEED_SCALE;

        fl.setPower(flp);
        fr.setPower(frp);
        bl.setPower(blp);
        br.setPower(brp);
    }

    // =========================
    // Turret + limit helpers
    // =========================
    private int turretPos() {
        return motorOne.getCurrentPosition();
    }

    private double applySoftLimitScaling(double power) {
        // OPTIONAL: slows down near ends so you don't smack the hard stop
        int pos = turretPos();

        if (power > 0) { // moving toward LEFT limit
            int dist = LIMIT_LEFT - pos; // ticks remaining
            if (dist <= 0) return 0;
            if (dist < SOFT_ZONE_TICKS) {
                double scale = clamp(dist / (double) SOFT_ZONE_TICKS, 0.0, 1.0);
                return power * scale;
            }
        } else if (power < 0) { // moving toward RIGHT limit
            int dist = pos - LIMIT_RIGHT; // ticks remaining
            if (dist <= 0) return 0;
            if (dist < SOFT_ZONE_TICKS) {
                double scale = clamp(dist / (double) SOFT_ZONE_TICKS, 0.0, 1.0);
                return power * scale;
            }
        }

        return power;
    }

    /**
     * HARD STOP FIRST.
     * If the encoder says you're at/past the limit and you're commanding further into it,
     * turret power becomes 0 and returns immediately.
     */
    private void setTurretPowerLimited(double power) {
        int pos = turretPos();

        // HARD stops
        if (power > 0 && pos >= LIMIT_LEFT) {
            turret.setPower(0);
            return;
        }
        if (power < 0 && pos <= LIMIT_RIGHT) {
            turret.setPower(0);
            return;
        }

        // Soft scaling (optional)
        power = applySoftLimitScaling(power);

        // Clamp max
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        turret.setPower(power);
    }

    // Pick the "best" valid target (prefer TARGET_ID, else allow 24, then choose closest to center)
    private AprilTagDetection pickBestTarget(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;

        for (AprilTagDetection d : detections) {
            if (d == null) continue;

            boolean valid = (d.id == TARGET_ID || d.id == 24);
            if (!valid) continue;

            if (best == null) {
                best = d;
            } else {
                if (Math.abs(d.ftcPose.bearing) < Math.abs(best.ftcPose.bearing)) {
                    best = d;
                }
            }
        }
        return best;
    }

    @Override
    public void loop() {
        // Reset field-centric heading
        if (gamepad1.back) {
            headingOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        // Optional: reset turret encoder (use this to measure LIMITs)
        if (gamepad1.x) {
            motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        fieldCentricDrive();

        List<AprilTagDetection> dets = aprilTag.getDetections();
        AprilTagDetection target = pickBestTarget(dets);

        boolean targetFound = (target != null);

        if (targetFound) {
            double errorDeg = target.ftcPose.bearing; // + / - depending on camera setup
            telemetry.addData("Target", target.id);
            telemetry.addData("Bearing(deg)", errorDeg);

            if (Math.abs(errorDeg) <= deadband) {
                setTurretPowerLimited(0);
            } else {
                double power = errorDeg * kP;

                // If it turns the wrong way, flip this once:
                // power = -power;

                setTurretPowerLimited(power);
            }
        } else {
            // SEARCH: oscillate between limits using encoder
            telemetry.addData("Status", "Searching...");

            int pos = turretPos();

            if (searchingLeft) {
                // if at/over left limit, flip direction
                if (pos >= LIMIT_LEFT) searchingLeft = false;
                setTurretPowerLimited(+SEARCH_POWER);
            } else {
                // if at/under right limit, flip direction
                if (pos <= LIMIT_RIGHT) searchingLeft = true;
                setTurretPowerLimited(-SEARCH_POWER);
            }
        }

        telemetry.addData("TurretEnc(motorOne)", turretPos());
        telemetry.addData("LIMIT_LEFT", LIMIT_LEFT);
        telemetry.addData("LIMIT_RIGHT", LIMIT_RIGHT);
        telemetry.addData("searchingLeft", searchingLeft);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        turret.setPower(0);
    }
}
