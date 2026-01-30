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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Disabled
@Config//important
@TeleOp
public class DecodeDriveRapidNo extends OpMode {
    private PIDFController controller;//important

    //revolver tuning
    public static double p = 0.1, i = 0, d= 0.0002;
    public static double f = 0.0001 ;
    private static final int home = 0;
    public static int target = home;//this number can be used for the shoot position.
    public static int intake = 96;
    public static int shoot = 48;
    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    //Shooter tunig
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
    ElapsedTime timer = new ElapsedTime();
    boolean timerStarted = false;



    public static double driveTrainPower = 0.8;

    //distance tuning
    public static double angleDegree = 0.49;//robot specific
    public static double lensHight = 13.5;//robot specific
    public static double goalHight  = 9;//allways use
    public static double Dvalue = 9;
    public static double Tvalue = 9;

    //turret Tuning
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

        leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        shooterB = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterT = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);


        Intake = hardwareMap.get(DcMotorEx.class, "intake");

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);


        status = "Initialized";
    }

    @Override
    public void start() {
        limelight.start();
        pidTimer.reset();
        timer.reset();
    }

    @Override
    public void loop(){
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

    public void Controls(){


        controller.setPIDF(p, i, d, f);
        int revpose = Revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);//math
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;//math

        controller.setTolerance(0.5);//makes more accurete
        controller.atSetPoint();//this always paired with setTolerance
        double power = pid + ff;//math that sets the power
        Revolver.setPower(power);
        telemetry.addData("pose1",revpose);

        if (gamepad1.xWasPressed()) {
            if (target == 0|| target == 144 || target == 240) {
                target = intake;
            } else if (target == 96) {
                target = 192;
            } else {
                target = home;
            }
        }

        if (gamepad1.yWasPressed()) {
            if (target == 0|| target == 96 || target == 192) {
                target = 144;
            } else if (target == 144) {
                target = 240;

            }else {
                target = shoot;
            }
        }

        if (gamepad1.dpadLeftWasPressed()) Intake.setPower(1);
        if (gamepad1.dpadRightWasPressed())Intake.setPower(0);


        if (gamepad1.dpadUpWasPressed()) {
            arm.setPosition(0.3);
        }

        if (gamepad1.dpadDownWasPressed()) {
            arm.setPosition(0);
        }

        if(gamepad2.a) {

            shooterT.setVelocity(pshot);
            shooterB.setVelocity(pshot);
            if (timer.seconds() == 2) {
                Revolver.setTargetPosition(96);
            }
            if (timer.seconds() == 2.5) {
                arm.setPosition(0.3);
                arm.setPosition(0);
            }
            if (timer.seconds() == 3) {
                Revolver.setTargetPosition(144);
            }
            if (timer.seconds() == 3.5) {
                arm.setPosition(0.3);
                arm.setPosition(0);
            }
            if (timer.seconds() == 4) {
                Revolver.setTargetPosition(288);
            }
            if(timer.seconds() == 4.5) {
                arm.setPosition(0.3);
                arm.setPosition(0);
                if (timer.seconds() == 6) {

                    shooterT.setVelocity(0);
                    shooterB.setVelocity(0);
                }
            }

        }

        if(gamepad1.bWasPressed()) {
            timer.reset();
            shooterT.setVelocity(fshot);
            shooterB.setVelocity(fshot);
            if (timer.seconds() == 2) {
                Revolver.setTargetPosition(96);
            }
            if (timer.seconds() == 2.5) {
                arm.setPosition(0.3);
                arm.setPosition(0);
            }
            if (timer.seconds() == 3) {
                Revolver.setTargetPosition(144);
            }
            if (timer.seconds() == 3.5) {
                arm.setPosition(0.3);
                arm.setPosition(0);
            }
            if (timer.seconds() == 4) {
                Revolver.setTargetPosition(288);
            }
            if (timer.seconds() == 4.5) {
                arm.setPosition(0.3);
                arm.setPosition(0);
                if (timer.seconds() == 6) {

                    shooterT.setVelocity(0);
                    shooterB.setVelocity(0);
                }

            }
        }
    }
    private void distanceLogic(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

        }

        double Y = llResult.getTy();
        double limelightMountAngleDegrees = angleDegree;//how angled it is

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = lensHight;

        // distance from the target to the floor
        double goalHeightInches = goalHight;

        //more math that i don't under stand that is grade 12 level
        double angleToGoalDegrees = limelightMountAngleDegrees + Y;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);//make sure the code is using math tan

        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

        if (gamepad2.right_bumper) {
            if (distance >= 80) {
                shooterT.setVelocity(pshot);
                shooterB.setVelocity(pshot);
            }

            if (distance <= 68) {
                shooterT.setVelocity(fshot);
                shooterB.setVelocity(fshot);
            }

            telemetry.addData("Distance",distance);
        }



    }

    public void DriveInit() {
        /*
        double leftJoyStickXAxis = gamepad1.left_stick_x * 1.1; //1.1 use to counteract imperfect strafing
        double leftJoyStickYAxis = -gamepad1.left_stick_y; //y stick value is reversed
        double rightJoyStickXAxis = gamepad1.right_stick_x;
        double botOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotateX = leftJoyStickXAxis * Math.cos(-botOrientation) - leftJoyStickYAxis * Math.sin(-botOrientation);
        double rotateY = leftJoyStickXAxis * Math.sin(-botOrientation) + leftJoyStickYAxis * Math.cos(-botOrientation);

        rotateX = rotateX * 1.1;  // Counteract imperfect strafing
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(leftJoyStickYAxis) + Math.abs(leftJoyStickXAxis) + Math.abs(rightJoyStickXAxis), 1);
        double frontLeftMotorPower = (rotateY + rotateX + rightJoyStickXAxis) / denominator;
        double backLeftMotorPower = (rotateY - rotateX + rightJoyStickXAxis) / denominator;
        double frontRightMotorPower = (rotateY - rotateX - rightJoyStickXAxis) / denominator;
        double backRightMotorPower = (rotateY + rotateX - rightJoyStickXAxis) / denominator;

        //Set motor power
        leftFront.setPower(frontLeftMotorPower * driveTrainPower);
        leftBack.setPower(backLeftMotorPower * driveTrainPower);
        rightBack.setPower(backRightMotorPower * driveTrainPower);
        rightFront.setPower(frontRightMotorPower * driveTrainPower);


        //IMU Reset
        if (gamepad1.back) {
            imu.resetYaw();
        }
         */
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
        telemetry.addData("Velocity",shooterB.getVelocity());
        telemetry.addData("Velocity2",shooterT.getVelocity());
        telemetry.update();
    }
}


