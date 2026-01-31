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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Config//important
@TeleOp//works
public class DecodeDrive extends OpMode {
    private PIDFController controller;//important

    public static double p = 0.1, i = 0, d= 0.0002;
    public static double f = 0.0001 ;

    private static final int home = 0;

    public static int target = home;//this number can be used for the shoot position.

    public static int intake = 96;

    public static int shoot = 48;

    public static double ARM_UP = 0.3, ARM_DOWN = 0.0;
    private ElapsedTime armTimer = new ElapsedTime();
    private boolean armMovingAuto = false;

    public static double HighVelocityShot = 1500;
    public static double LowVelocityShot = 1300;
    public double curTargetVelocity = HighVelocityShot;
    public static double F = 17.5;
    public static double P = 13;

    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    private DcMotorEx Revolver;
    private CRServo turretServo;
    private Servo arm;
    private DcMotorEx shooterT;
    private DcMotorEx shooterB;
    private Limelight3A limelight;
    private IMU imu;
    private DcMotorEx Intake;
    private GoBildaPinpointDriver pinpoint;
    private TouchSensor touch;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    private int lockedTargetID = -1;


    public static int Pollher = 100;


    public static int TARGET_ID = 24;
    public static double Lp = 0.02;
    public static double Ld = 0.002;
    public static double MaxPower = 0.5;
    public static double MinPower = 0.05;
    public static double Tolerance = 0.5;

    // Safety Limits
    public static boolean Limits = true;
    public static int MinPo = -11000;
    public static int Maxpo = 3600;


    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private String status = "Initializing";


    @Override
    public void init(){
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//allow to do stuff in dash board
        Revolver = hardwareMap.get(DcMotorEx.class,"revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);//better stopping
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);//it better to run without encoders because it is faster
        final int home = 0;


        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientation));


        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretServo.setDirection(CRServo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(Pollher);
        limelight.pipelineSwitch(0);

        leftFront = hardwareMap.get(DcMotor.class, "Fl");
        rightFront = hardwareMap.get(DcMotor.class, "Fr");
        rightBack = hardwareMap.get(DcMotor.class, "Br");
        leftBack = hardwareMap.get(DcMotor.class, "Bl");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        shooterT = hardwareMap.get(DcMotorEx.class,"shooterT");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");

        shooterB = hardwareMap.get(DcMotorEx.class,"shooterB");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P,0,0,F);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        telemetry.addLine("init complete");

        Intake = hardwareMap.get(DcMotorEx.class, "intake");

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);

        touch = hardwareMap.get(TouchSensor.class, "touch");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        configurePinpoint();
        status = "Initialized";
    }

    @Override
    public void start() {
        limelight.start();
        pidTimer.reset();
    }

    @Override
    public void loop(){
        runTurretLogic();
        DriveInit();


        controller.setPIDF(p, i, d, f);
        int revpose = Revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);//math
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;//math

        controller.setTolerance(0.5);//makes more accurete
        controller.atSetPoint();//this always paired with setTolerance
        double power = pid + ff;//math that sets the power
        Revolver.setPower(power);
        telemetry.addData("pose1",revpose);
        boolean pressed = touch.isPressed();

        if (gamepad2.xWasPressed()) {
            if (target == 0|| target == 144 || target == 240) {
                target = intake;
            } else if (target == 96) {
                target = 192;
            } else {
                target = home;
            }
        }
        /*
        if (gamepad1.dpad_left) {
            Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.dpad_right) {
            Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }


         */




        if (gamepad2.yWasPressed()) {
            if (target == 0|| target == 96 || target == 192 || target == 48) {
                target = 144;
            } else if (target == 144) {
                target = 240;

            }else {
                target = shoot;
            }
        }
        if (gamepad1.a) {
            curTargetVelocity = 4000;
        }

        if (gamepad1.y) {
            curTargetVelocity = 3500;
        }

        if (gamepad2.dpadLeftWasPressed()) Intake.setPower(1);
        if (gamepad2.dpadRightWasPressed())Intake.setPower(0);

        if (gamepad2.right_trigger >= 1) {
            Intake.setPower(-1);
        }
        if (gamepad2.dpadUpWasPressed()) {
            armMovingAuto = true;
            armTimer.reset();
        }

        if (armMovingAuto) {
            if (armTimer.seconds() < 0.4) {
                arm.setPosition(ARM_UP);
            } else if (armTimer.seconds() < 0.8) {
                arm.setPosition(ARM_DOWN);
            } else {
                armMovingAuto = false;
            }
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P,0, 0, F);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);


        if (gamepad2.right_bumper) {

                shooterT.setVelocity(curTargetVelocity);
                shooterB.setVelocity(curTargetVelocity);
            }

        if (gamepad2.left_bumper){
                shooterT.setVelocity(LowVelocityShot);
                shooterB.setVelocity(LowVelocityShot);
            }

        if (gamepad2.b){
            shooterT.setVelocity(0);
            shooterB.setVelocity(0);
        }



        updateTelemetry();


    }


    @Override
    public void stop() {
        limelight.stop();
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

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        Pose2D pose2D = pinpoint.getPosition();
    }

    public void configurePinpoint(){

        pinpoint.setOffsets(76.2, 127, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
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

        // these are pid constrant
        if (Math.abs(output) < MinPower) {
            output = Math.signum(output) * MinPower;
        }
        return Math.max(-MaxPower, Math.min(MaxPower, output));
    }

    private void updateTelemetry() {
        telemetry.addData("Status", status);
        telemetry.addData("Turret Pos", leftFront.getCurrentPosition());
        telemetry.addData("Turret Power", turretServo.getPower());
        telemetry.addData("Target",target);
        telemetry.update();
    }
}


