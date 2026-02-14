package org.firstinspires.ftc.teamcode.NewRo2.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@Config
@TeleOp
public class OpModeTurret extends OpMode {


    private CRServo turretServo;
    private DcMotorEx externalEncoder;
    private Limelight3A limelight;
    private IMU imu;


    public static int TARGET_ID = 24;
    public static double p = 0.02;
    public static double d = 0.002;
    public static double MaxPower = 0.5;
    public static double MinPower = 0.05;
    public static double Tolerance = 0.5;

    // Safety Limits
    public static boolean Limits = true;
    public static int MinPo = -3000000;
    public static int Maxpo = 3000000;


    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private String status = "Initializing";

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretServo = hardwareMap.get(CRServo.class, "t");
        externalEncoder = hardwareMap.get(DcMotorEx.class, "E");
        externalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        externalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);


        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientation));

        status = "Initialized";
    }

    @Override
    public void start() {
        limelight.start();
        pidTimer.reset();
    }

    @Override
    public void loop() {
        runTurretLogic();
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

        if (llResult != null && llResult.isValid()) {
            double TX = llResult.getTx();
            double power = -calculatePID(TX);

            // Safety Limits check
            int currentPos = externalEncoder.getCurrentPosition();
            if (Limits) {
                if (currentPos >= Maxpo && power > 0) power = 0;
                else if (currentPos <= MinPo && power < 0) power = 0;
            }

            turretServo.setPower(power);
            status = "Tracking Target " + TARGET_ID;
        } else {
            turretServo.setPower(0);
            status = "Searching...";
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

        double P = p * error;
        double D = d * (error - lastError) / deltaTime;
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
        telemetry.addData("Turret Pos", externalEncoder.getCurrentPosition());
        telemetry.addData("Turret Power", turretServo.getPower());
        telemetry.update();
    }
}