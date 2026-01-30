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
public class DecodeDrive2 extends OpMode {
    private PIDController controller;

    // --- Revolver Tuning ---
    public static double p = 0.1, i = 0, d = 0.0002, f = 0.0001;
    private final double ticks_in_degree = 700 / 180.0;
    public static int target = 0;
    private static final int home = 0;
    public static int intake = 96;
    public static int shoot = 48;
    private static final int[] SLOT_POSITIONS = new int[]{0, 96, 192};
    private int slotIndex = 0;

    // --- Shooter Tuning ---
    public static double pshot = 7.3013, ishot = 0, dshot = 0, fshot = 10;
    public static double HighVelocityShot = 5000;
    public static double LowVelocityShot = 900;

    // --- State Machine Fields ---
    private enum RapidState { IDLE, GO_SHOOT, ARM_UP, ARM_DOWN, INDEX_NEXT, RETURN_INTAKE }
    private RapidState rapidState = RapidState.IDLE;
    private enum RapidMode { NONE, LOW, HIGH }
    private RapidMode rapidMode = RapidMode.NONE;

    private ElapsedTime rapidTimer = new ElapsedTime();
    private int rapidShots = 0;
    private int ballCount = 0;
    private boolean lastTouchPressed = false;
    private boolean touchIndexEnabled = true;

    // --- Constants ---
    public static double ARM_UP_POS = 0.30, ARM_DOWN_POS = 0.00;
    public static double ARM_UP_TIME = 0.12, ARM_DOWN_TIME = 0.12;
    public static double INDEX_TIME = 0.18, SHOOT_SETTLE_TIME = 0.10, RETURN_SETTLE_TIME = 0.20;

    // --- Hardware ---
    private DcMotorEx Revolver, Intake, shooterT, shooterB;
    private CRServo turretServo;
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor leftFront, rightFront, rightBack, leftBack;
    private Servo arm;
    private GoBildaPinpointDriver pinpoint;
    private TouchSensor touch;

    // --- Turret / Distance Tuning ---
    public static double angleDegree = 0.49, lensHight = 13.5, goalHight = 9;
    public static double Lp = 0.02, Ld = 0.002, MaxPower = 0.5, MinPower = 0.05, Tolerance = 0.5;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    public static boolean Limits = true;
    public static int MinPo = -11000, Maxpo = 3600;
    private String status = "Initializing";
    private int lockedTargetID = -1;

    public static double driveTrainPower = 0.8;

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
        configurePinpoint();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        leftFront = hardwareMap.get(DcMotor.class, "Fl");
        rightFront = hardwareMap.get(DcMotor.class, "Fr");
        rightBack = hardwareMap.get(DcMotor.class, "Br");
        leftBack = hardwareMap.get(DcMotor.class, "Bl");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(Servo.class, "arm");
        touch = hardwareMap.get(TouchSensor.class, "touch");

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

        // 1. Determine Rapid Fire State
        if (gamepad1.right_bumper) rapidMode = RapidMode.LOW;
        else if (gamepad1.left_bumper) rapidMode = RapidMode.HIGH;

        // 2. Handle Logic Based on Mode
        if (rapidMode != RapidMode.NONE) {
            rapidFireSequence(rapidMode == RapidMode.LOW ? LowVelocityShot : HighVelocityShot);
        } else {
            Controls(); // Manual and Touch Sensor logic
        }

        distanceLogic();
        updateRevolverPID(); // Move motor at the end of loop
        updateTelemetry();
    }

    private void updateRevolverPID() {
        controller.setPID(p, i, d);
        controller.setTolerance(1.0); // Allow 1 tick error to prevent oscillating

        int currentPos = Revolver.getCurrentPosition();
        double pidOutput = controller.calculate(currentPos, target);
        double feedforward = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        // FIX: Deadband check. If the PID says we are at the target, force power to 0.
        if (controller.atSetPoint()) {
            Revolver.setPower(0);
        } else {
            Revolver.setPower(pidOutput + feedforward);
        }
    }

    private void rapidFireSequence(double velocity) {
        touchIndexEnabled = false;
        shooterT.setVelocity(velocity);
        shooterB.setVelocity(velocity);

        if (rapidState == RapidState.IDLE) {
            rapidShots = 0;
            rapidTimer.reset();
            rapidState = RapidState.GO_SHOOT;
            status = "RapidFire Active";
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
                    rapidState = (rapidShots >= 3) ? RapidState.RETURN_INTAKE : RapidState.INDEX_NEXT;
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
                    touchIndexEnabled = true;
                    rapidState = RapidState.IDLE;
                    rapidMode = RapidMode.NONE; // Return to manual mode
                    status = "Ready";
                }
                break;
        }
    }

    public void Controls() {
        // Intake
        if (gamepad1.dpad_right) Intake.setPower(0);
        else if (gamepad1.right_trigger > 0.1) Intake.setPower(1);
        else if (gamepad1.left_trigger > 0.1) Intake.setPower(-1);

        // Arm
        if (gamepad2.dpad_up) arm.setPosition(0.3);
        if (gamepad2.dpad_down) arm.setPosition(0);

        // Touch Sensor Indexing
        if (touchIndexEnabled) {
            boolean touchPressed = touch.isPressed();
            if (touchPressed && !lastTouchPressed) {
                ballCount++;
                if (ballCount >= 3) {
                    target = shoot;
                } else {
                    slotIndex = (slotIndex + 1) % SLOT_POSITIONS.length;
                    target = SLOT_POSITIONS[slotIndex];
                }
            }
            lastTouchPressed = touchPressed;
        }

        if (gamepad2.a) {
            ballCount = 0;
            slotIndex = 0;
            target = SLOT_POSITIONS[slotIndex];
        }
    }

    private void distanceLogic() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return;

        double angleToGoalRadians = (angleDegree + llResult.getTy()) * (Math.PI / 180.0);
        double distance = (goalHight - lensHight) / Math.tan(angleToGoalRadians);

        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

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
        boolean specificTargetVisible = false;

        // Check if we see valid targets
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();


            if (lockedTargetID == -1) {
                if (!fiducialResults.isEmpty()) {

                    lockedTargetID = fiducialResults.get(0).getFiducialId();
                    status = "LOCKED onto ID: " + lockedTargetID;
                }
            }


            if (lockedTargetID != -1) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == lockedTargetID) {
                        if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                            double TX = fr.getTargetXDegrees();
                            power = calculatePID(TX);
                            specificTargetVisible = true;
                            break;
                        }
                    }
                }
            }
        }


        if (specificTargetVisible) {

            if (Limits) {

                int currentPos = leftFront.getCurrentPosition();
                if (currentPos >= Maxpo && power > 0) {
                    power = 0;
                } else if (currentPos <= MinPo && power < 0) {
                    power = 0;
                }
            }
            turretServo.setPower(power);
            status = "Tracking ID " + lockedTargetID;
        } else {

            turretServo.setPower(0);
            if (lockedTargetID != -1) {
                status = "Searching for ID " + lockedTargetID + "...";
            } else {
                status = "Waiting for any target...";
            }
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
        telemetry.addData("Revolver Target", target);
        telemetry.addData("Ball Count", ballCount);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}