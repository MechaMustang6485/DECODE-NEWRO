package org.firstinspires.ftc.teamcode.Pratice;






import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp
public class GoodTurret extends LinearOpMode {

    DcMotor leftFront;

    DcMotor rightFront;

    DcMotor rightBack;

    DcMotor leftBack;

    //Good turret please use it
    private CRServo turretServo;
    private DcMotorEx Intake;
    private Limelight3A limelight;
    private IMU imu;

    public static int TARGET_ID = 24;

    public static int pollher = 100;

    public static double p = 0.02;
    public static double d = 0.001;

    public static double MaxPower = 1;
    public static double MinPower = 0.05;
    public static double Tolerance = 0.5;

   public static double driveTrainPower = 0.8;

    // Safety Limits
    public static boolean Limits = true;
    public static int MinPo = -11000;
    public static int Maxpo = 3600;

    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private String status = "Initializing";



    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initHardware();

        while (opModeInInit()) {
            telemetry.addData("Status", "Limelight Ready");
            telemetry.addData("Position", leftFront.getCurrentPosition());//the position shoud be at 0
            telemetry.update();
        }

        limelight.start();
        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            imuDriveInit();
            runTurretLogic();
            control();
            updateTelemetry();
        }

        limelight.stop();
    }



    public void imuDriveInit() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

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
    }

    private void control(){
        if (gamepad1.dpadLeftWasPressed()) Intake.setPower(1);
        if (gamepad1.dpadRightWasPressed())Intake.setPower(0);
    }


    private void runTurretLogic() {


        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

        }


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


    //if you are making your own turret code copy this part.
    //////////////////////////////////////////////////////////////////////////////////////////////////
    private double calculatePID(double error) {
        //Caculate the amount of time for the movement
        double deltaTime = pidTimer.seconds();
        if (deltaTime == 0) deltaTime = 0.02;
        pidTimer.reset();

        if (Math.abs(error) < Tolerance) {
            lastError = 0;
            return 0;//stops the Servo if the tolorence is to high, you do not need to add .atSetPoint().
        }

        //more Caculation
        double P = p * error;
        double D = d * (error - lastError) / deltaTime;
        lastError = error;

        double output = P + D;

        if (Math.abs(output) < MinPower) {
            output = Math.signum(output) * MinPower;
        }

        return Math.max(-MaxPower, Math.min(MaxPower, output));
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void initHardware() {

        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretServo.setDirection(CRServo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(pollher);
        limelight.pipelineSwitch(0);

        leftFront = hardwareMap.get(DcMotor.class, "Fl"); // Control Hub Motor 3
        rightFront = hardwareMap.get(DcMotor.class, "Fr"); // Control Hub Motor 1
        rightBack = hardwareMap.get(DcMotor.class, "Br"); // Control Hub Motor 0
        leftBack = hardwareMap.get(DcMotor.class, "Bl"); // Control Hub Motor 2

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotorEx.class, "intake");

    }

    private void updateTelemetry() {
        telemetry.addData("Status", status);
        telemetry.addData("Turret Pos", leftFront.getCurrentPosition());
        telemetry.addData("Power", "%.2f", turretServo.getPower());
        telemetry.addData("Limit Hit Max", leftFront.getCurrentPosition() >= Maxpo);
        telemetry.addData("Limit Hit Min", leftFront.getCurrentPosition() <= MinPo);
        telemetry.update();
    }
}