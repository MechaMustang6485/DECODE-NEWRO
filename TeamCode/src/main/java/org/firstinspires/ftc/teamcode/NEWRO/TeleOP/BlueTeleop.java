package org.firstinspires.ftc.teamcode.NEWRO.TeleOP;






import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TeleopRev;


import java.util.List;

@Config
@TeleOp
public class BlueTeleop extends OpMode {

    //shooter tuning
    public static double HighVelocityShot = 1500;
    public static double LowVelocityShot = 1225;
    public double curTargetVelocity = HighVelocityShot;
    public static double F = 17.5;//15
    public static double P = 200;

    //Rapid shooting
    public static double ARM_UP = 0.3, ARM_DOWN = 0.0;
    private ElapsedTime RevolverTimer = new ElapsedTime();
    private boolean RevovlerMoving = false;


    //parts of robot
    private CRServo turretServo;
    private Servo arm;
    private DcMotorEx shooterT;
    private DcMotorEx shooterB;
    private Limelight3A limelight;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    TeleopRev Revolver;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor Intake;

    //turret
    private int lockedTargetID = -1;
    public static int TARGET_ID = 24;
    public static double Lp = 0.02;
    public static double Ld = 0.002;
    public static double MaxPower = 0.5;
    public static double MinPower = 0.05;
    public static double Tolerance = 0.5;

    // Safety Limits
    public static boolean Limits = true;
    public static int MinPo = -11000;
    public static int Maxpo = 4800;


    //do not touch, for turret
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private String status = "Initializing";



    @Override
    public void init(){

        Revolver = new TeleopRev(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientation));

        Intake = hardwareMap.get(DcMotor.class, "intake");


        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretServo.setDirection(CRServo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        leftFront = hardwareMap.get(DcMotor.class, "Fl");
        rightFront = hardwareMap.get(DcMotor.class, "Fr");
        rightBack = hardwareMap.get(DcMotor.class, "Br");
        leftBack = hardwareMap.get(DcMotor.class, "Bl");

        //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);


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
        Revolver.update();

        if (gamepad2.yWasPressed()) {
            RevovlerMoving = true;
            RevolverTimer.reset();
        }

        if (RevovlerMoving) {//this works
            double elapsed = RevolverTimer.seconds();
            if (elapsed < 1.3) {
                Revolver.setTargetPosition(240);
                if (elapsed > 0.6 && elapsed < 1) arm.setPosition(ARM_UP);
                else arm.setPosition(ARM_DOWN);
            }
            else if (elapsed < 2.4) {
                Revolver.setTargetPosition(144);
                if (elapsed > 1.7 && elapsed < 2.2) arm.setPosition(ARM_UP);
                else arm.setPosition(ARM_DOWN);
            }
            else if (elapsed < 3.9) {
                Revolver.setTargetPosition(48);
                if (elapsed > 3 && elapsed < 3.5) arm.setPosition(ARM_UP);
                else arm.setPosition(ARM_DOWN);
            }
            else {
                RevovlerMoving = false;
                arm.setPosition(ARM_DOWN);
                Revolver.goToSlot(0);
             Revolver.IntakePower(1);
                shooterT.setVelocity(900);
                shooterB.setVelocity(900);
            }
        }
/*
        if (gamepad2.yWasPressed()) {//shoot position
            if (target == 0|| target == 96 || target == 192 || target == 48) {
                target = 144;
            } else if (target == 144) {
                target = 240;

            }else {
                target = shoot;
            }
        }
    if (gamepad2.xWasPressed()) {
            if (target == 0|| target == 144 || target == 240) {
                target = intake;
            } else if (target == 96) {
                target = 192;
            } else {
                target = home;
            }

 */
        if (gamepad2.xWasPressed()) {
            if (Revolver.getTarget() == 0) {
                Revolver.setTargetPosition(96);
            } else if (Revolver.getTarget() == 96) {
                Revolver.setTargetPosition(192);
            } else if (Revolver.getTarget() == 192) {
                Revolver.setTargetPosition(288);
            } else {
                Revolver.goToSlot(0);
            }
        }
        if (gamepad2.aWasPressed()) {
            if (Revolver.getTarget() == 0||Revolver.getTarget() == 96||Revolver.getTarget() == 192||Revolver.getTarget() == 288) {
                Revolver.setTargetPosition(48);
            } else if (Revolver.getTarget() == 48) {
                Revolver.setTargetPosition(144);
            } else if (Revolver.getTarget() == 144) {
                Revolver.setTargetPosition(240);
            } else {
                Revolver.goToSlot(0);
            }
        }


        if (gamepad2.dpadLeftWasPressed())    Intake.setPower(1);
        if (gamepad2.dpadRightWasPressed()) Intake.setPower(0);

        if (gamepad2.right_trigger >= 1) {
            Revolver.IntakePower(-1);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P,0, 0, F);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

        if (gamepad2.right_bumper) {
            shooterT.setVelocity(curTargetVelocity);
            shooterB.setVelocity(curTargetVelocity);
            Revolver.IntakePower(0);
        }

        if (gamepad2.left_bumper){
            shooterT.setVelocity(LowVelocityShot);
            shooterB.setVelocity(LowVelocityShot);
            Revolver.IntakePower(0);
        }

        if (gamepad2.b){
            shooterT.setVelocity(0);
            shooterB.setVelocity(0);
            Revolver.IntakePower(1);
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


        if (llResult != null) {


            double TX = llResult.getTx();
            double power = calculatePID(TX);

            // Safety Limits
            int currentPos = leftFront.getCurrentPosition();//the encoder make thing more accuret
            if (Limits) {
                if (currentPos >= Maxpo && power > 0){
                    power = 0;
                }

                else if (currentPos <= MinPo && power < 0){
                    power = 0;
                };
            }

            turretServo.setPower(power);
            status = "Sees" + TARGET_ID;
        } else {
            turretServo.setPower(0);
            status = "Searching";
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
        telemetry.addData("Encoder", Revolver.getEncoder());
        telemetry.addData("target", Revolver.getTarget());
        telemetry.addData("velocity1", shooterT.getVelocity());
        telemetry.addData("velocity2", shooterB.getVelocity());
        telemetry.update();
    }
}