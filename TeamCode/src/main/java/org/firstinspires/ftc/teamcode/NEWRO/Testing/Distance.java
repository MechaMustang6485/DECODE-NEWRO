package org.firstinspires.ftc.teamcode.NEWRO.Testing;








import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@Config
@TeleOp
public class Distance extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private DcMotorEx shooterT;
    private DcMotorEx shooterB;

    private Servo arm;

    public static double angleDegree = 0.49;//robot specific
    public static double lensHight = 13.5;//robot specific
    public static double goalHight  = 9;//allways use

    public static double Dvalue = 9;
    public static double Tvalue = 9;

    private PIDFController controller;//important

    public static double p = 0.1, i = 0, d= 0.0002;
    public static double f = 0.0001 ;

    private static final int home = 0;

    public static int target = home;//this number can be used for the shoot position.

    public static int intake = 96;

    public static int shoot = 48;


    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    private DcMotorEx Revolver;






    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        Revolver = hardwareMap.get(DcMotorEx.class,"revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);//better stopping
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);//it better to run without encoders because it is faster
        final int home = 0;

        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.FORWARD);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);


        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        controller.setPIDF(p, i, d, f);
        int revpose = Revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);//math
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;//math

        controller.setTolerance(0.5);//makes more accurete
        controller.atSetPoint();//this always paired with setTolerance
        double power = pid + ff;//math that sets the power
        Revolver.setPower(power);
        telemetry.addData("pose1",revpose);


        // Get orientation and Tx, Ty, Ta
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


        if (gamepad1.xWasPressed()) {
            if (target == 0) {
                target = intake;
            } else if (target == 96) {
                target = 192;
            } else {
                target = home;
            }
        }

        if (gamepad1.yWasPressed()) {
            if (target == 0) {
                target = 144;
            } else if (target == 144) {
                target = 240;

            }else {
                target = shoot;
            }
        }

        if (gamepad1.a){
            shooterB.setVelocity(distance * Tvalue);// you can change the equation to any number
            shooterT.setVelocity(distance * Dvalue);
        }

        if (gamepad1.dpadUpWasPressed()) {
            arm.setPosition(0.3);
        }

        if (gamepad1.dpadDownWasPressed()) {
            arm.setPosition(0);
        }


        telemetry.addData("Distance",distance);
        telemetry.addData("Velocity",shooterB.getVelocity());
        telemetry.addData("Velocity2",shooterT.getVelocity());

        telemetry.update();
    }



}

