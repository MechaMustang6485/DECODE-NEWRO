package org.firstinspires.ftc.teamcode.NEWRO.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
@Disabled
@Config
@TeleOp
public class DecodeDriveTestNo extends OpMode {
    private PIDFController controller;

    // revolver tuning
    public static double p = 0.1, i = 0, d = 0.0002;
    public static double f = 0.0001;
    private static final int home = 0;

    // Your existing positions
    public static int intake = 96;
    public static int shoot = 48;

    // 3-slot indexing positions (based on your old X cycle: 0 -> 96 -> 192 -> 0)
    private static final int[] SLOT_POSITIONS = new int[]{0, 96, 192};
    private int slotIndex = 0;

    // ball counting (touch sensor increments)
    private int ballCount = 0;
    private boolean lastTouchPressed = false;

    private final double ticks_in_degree = 700 / 180.0;

    // Shooter tuning
    public static double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 10;
    public static double HighVelocityShot = 5000;
    public static double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    private DcMotorEx Revolver;
    private CRServo turretServo;
    private Limelight3A limelight;
    private IMU imu;
    private DcMotorEx Intake;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private Servo arm;
    private DcMotorEx shooterT;
    private DcMotorEx shooterB;
    private GoBildaPinpointDriver pinpoint;

    // ===== 1) ADD THESE FIELDS (top of class) =====
    private enum RapidState { IDLE, GO_SHOOT, ARM_UP, ARM_DOWN, INDEX_NEXT, RETURN_INTAKE }
    private RapidState rapidState = RapidState.IDLE;

    private ElapsedTime rapidTimer = new ElapsedTime();
    private int rapidShots = 0;

    public static double ARM_UP_POS = 0.30;
    public static double ARM_DOWN_POS = 0.00;
    public static double ARM_UP_TIME = 0.12;
    public static double ARM_DOWN_TIME = 0.12;
    public static double INDEX_TIME = 0.18;
    public static double SHOOT_SETTLE_TIME = 0.10;
    public static double RETURN_SETTLE_TIME = 0.20;

    private enum RapidMode { NONE, LOW, HIGH }
    private RapidMode rapidMode = RapidMode.NONE;

    private boolean touchIndexEnabled = true;


    // NEW: touch sensor
    private TouchSensor touch;

    public static double driveTrainPower = 0.8;

    // distance tuning
    public static double angleDegree = 0.49;
    public static double lensHight = 13.5;
    public static double goalHight = 9;

    // turret tuning
    public static int Pollher = 100;
    public static int TARGET_ID = 24;
    public static double Lp = 0.02;
    public static double Ld = 0.002;
    public static double MaxPower = 0.5;
    public static double MinPower = 0.05;
    public static double Tolerance = 0.5;
    private int lockedTargetID = -1;

    // Safety Limits
    public static boolean Limits = true;
    public static int MinPo = -11000;
    public static int Maxpo = 3600;

    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private String status = "Initializing";

    // This is the single “revolver target” you drive with PID
    public static int target = home;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientation));

        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretServo.setDirection(CRServo.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        configurePinpoint();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        leftFront = hardwareMap.get(DcMotor.class, "Fl");
        rightFront = hardwareMap.get(DcMotor.class, "Fr");
        rightBack = hardwareMap.get(DcMotor.class, "Br");
        leftBack = hardwareMap.get(DcMotor.class, "Bl");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotorEx.class, "intake");

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);

        // NEW: touch sensor in config named "touch"
        touch = hardwareMap.get(TouchSensor.class, "touch");

        // init state
        slotIndex = 0;
        ballCount = 0;
        target = SLOT_POSITIONS[slotIndex];

        status = "Initialized";
    }

    @Override
    public void start() {
        limelight.start();
        pidTimer.reset();
    }

    @Override
    public void loop() {
        DriveInit();
        runTurretLogic();
        Controls();
        distanceLogic();
        updateTelemetry();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    // ===== 3) REPLACE rapidFireLowSequence() WITH THIS =====
    private void rapidFireSequence(double velocity) {
        touchIndexEnabled = false;

        shooterT.setVelocity(velocity);
        shooterB.setVelocity(velocity);

        if (rapidState == RapidState.IDLE) {
            rapidShots = 0;
            rapidTimer.reset();
            rapidState = RapidState.GO_SHOOT;
            status = "RapidFire " + (velocity == LowVelocityShot ? "LOW" : "HIGH");
        }

        switch (rapidState) {
            case GO_SHOOT:
                target = shoot;
                if (rapidTimer.seconds() > SHOOT_SETTLE_TIME) {
                    rapidTimer.reset();
                    rapidState = RapidState.ARM_UP;
                }
                break;

            case ARM_UP:
                arm.setPosition(ARM_UP_POS);
                if (rapidTimer.seconds() > ARM_UP_TIME) {
                    rapidTimer.reset();
                    rapidState = RapidState.ARM_DOWN;
                }
                break;

            case ARM_DOWN:
                arm.setPosition(ARM_DOWN_POS);
                if (rapidTimer.seconds() > ARM_DOWN_TIME) {
                    rapidTimer.reset();
                    rapidShots++;

                    if (rapidShots >= 3) rapidState = RapidState.RETURN_INTAKE;
                    else rapidState = RapidState.INDEX_NEXT;
                }
                break;

            case INDEX_NEXT:
                slotIndex = (slotIndex + 1) % SLOT_POSITIONS.length;
                target = SLOT_POSITIONS[slotIndex];

                if (rapidTimer.seconds() > INDEX_TIME) {
                    rapidTimer.reset();
                    rapidState = RapidState.GO_SHOOT;
                }
                break;

            case RETURN_INTAKE:
                shooterT.setVelocity(0);
                shooterB.setVelocity(0);
                target = intake;

                if (rapidTimer.seconds() > RETURN_SETTLE_TIME) {
                    ballCount = 0;
                    lastTouchPressed = false;
                    touchIndexEnabled = true;
                    rapidState = RapidState.IDLE;
                    status = "RapidFire done";
                }
                break;

            default:
                rapidState = RapidState.IDLE;
                break;
        }
    }



    public void Controls() {
        // -------- Revolver PID to target --------
        controller.setPIDF(p, i, d, f);

        int revpose = Revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        controller.setTolerance(0.5);
        controller.atSetPoint();

        double power = pid + ff;
        Revolver.setPower(power);
        telemetry.addData("Revolver Pos", revpose);

        // -------- Intake/Outtake controls (TRIGGERS + DPAD RIGHT STOP) --------
        // Right trigger = intake
        // Left trigger = outtake
        // Dpad right = stop
        if (gamepad1.dpad_right) {
            Intake.setPower(0);
        }

        if (gamepad1.right_trigger_pressed) {
            Intake.setPower(1);
        }

        if (gamepad1.left_trigger_pressed) {
            Intake.setPower(-1);
        }

        // -------- Arm controls (kept as-is) --------
        if (gamepad2.dpadUpWasPressed()) {
            arm.setPosition(0.3);
        }
        if (gamepad2.dpadDownWasPressed()) {
            arm.setPosition(0);
        }

        // ===== 2) TOUCH SENSOR SNIPPET (wrap your existing touch logic) =====
        if (touchIndexEnabled) {
            boolean touchPressed = touch.isPressed();

            if (touchPressed && !lastTouchPressed) {
                ballCount++;

                if (ballCount >= 3) {
                    ballCount = 3;
                    target = shoot;
                    status = "3 balls: going to SHOOT";
                } else {
                    slotIndex = (slotIndex + 1) % SLOT_POSITIONS.length;
                    target = SLOT_POSITIONS[slotIndex];
                    status = "Indexed to slot " + (slotIndex + 1) + " (balls=" + ballCount + ")";
                }
            }

            lastTouchPressed = touchPressed;
        }


        // OPTIONAL: quick reset if you want (not required, but useful)
        // A = reset ball count + go home
        if (gamepad2.aWasPressed()) {
            ballCount = 0;
            slotIndex = 0;
            target = SLOT_POSITIONS[slotIndex];
            status = "Reset: ballCount=0, slot=1";
        }
    }

    private void distanceLogic() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            return; // IMPORTANT: prevents null crash
        }

        Pose3D botPose = llResult.getBotpose_MT2();
        telemetry.addData("Tx", llResult.getTx());
        telemetry.addData("Ty", llResult.getTy());
        telemetry.addData("Ta", llResult.getTa());

        double Y = llResult.getTy();

        double limelightMountAngleDegrees = angleDegree;
        double limelightLensHeightInches = lensHight;
        double goalHeightInches = goalHight;

        double angleToGoalDegrees = limelightMountAngleDegrees + Y;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

        if (gamepad1.right_bumper) {
            rapidMode = RapidMode.LOW;
            rapidFireSequence(LowVelocityShot);
        }

        if (gamepad1.leftBumperWasPressed()) {
            rapidMode = RapidMode.HIGH;
            rapidFireSequence(HighVelocityShot);
        }

        }

    public void DriveInit() {
        pinpoint.update();
        if (gamepad1.back) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            telemetry.addData("IMU Status", "Yaw Reset Initiated!");
        }

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

        leftFront.setPower(frontLeftPower * driveTrainPower);
        leftBack.setPower(backLeftPower * driveTrainPower);
        rightFront.setPower(frontRightPower * driveTrainPower);
        rightBack.setPower(backRightPower * driveTrainPower);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(76.2, 127, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    private void runTurretLogic() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        double power = 0;
        boolean targetFound = false;

        int allowedID = -1;
        if (gamepad2.right_bumper) {
            allowedID = 20;
        } else if (gamepad2.left_bumper) {
            allowedID = 24;
        }

        if (llResult != null && llResult.isValid() && allowedID != -1) {
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                if ( fr.getFiducialId() == allowedID) {
                    double TX = fr.getTargetXDegrees();
                    power = calculatePID(TX);
                    lockedTargetID = allowedID;
                    targetFound = true;
                    break;
                }
            }
        }

        if (targetFound) {
            // Safety Limits check
            if (Limits) {
                int currentPos = leftFront.getCurrentPosition();
                if (currentPos >= Maxpo && power > 0) power = 0;
                else if (currentPos <= MinPo && power < 0) power = 0;
            }
            turretServo.setPower(power);
            status = "Locked on ID: " + allowedID;
        } else {
            turretServo.setPower(0);
            lockedTargetID = -1;

            // Dynamic status message
            if (allowedID == -1) status = "Hold a bumper to track";
            else status = "Searching for ID: " + allowedID;
        }
    }

    private double calculatePID(double error) {
        double deltaTime = pidTimer.seconds();
        if (deltaTime == 0) deltaTime = 0.02;
        pidTimer.reset();

        if (Math.abs(error) < Tolerance) {
            lastError = 0;
            return 0;
        }

        double P = Lp * error;
        double D = Ld * (error - lastError) / deltaTime;
        lastError = error;

        double output = P + D;

        if (Math.abs(output) < MinPower) {
            output = Math.signum(output) * MinPower;
        }
        return Math.max(-MaxPower, Math.min(MaxPower, output));
    }

    private void updateTelemetry() {
        telemetry.addData("Status", status);
        telemetry.addData("Turret Pos", leftFront.getCurrentPosition());
        telemetry.addData("Turret Power", turretServo.getPower());

        telemetry.addData("Revolver Target", target);
        telemetry.addData("Slot Index", slotIndex + 1);
        telemetry.addData("Ball Count", ballCount);

        telemetry.addData("Shooter Vel B", shooterB.getVelocity());
        telemetry.addData("Shooter Vel T", shooterT.getVelocity());
        telemetry.update();
    }
}
