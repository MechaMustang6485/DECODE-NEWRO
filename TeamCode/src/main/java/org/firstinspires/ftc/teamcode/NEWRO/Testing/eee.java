package org.firstinspires.ftc.teamcode.NEWRO.Testing;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled

@Config
@TeleOp
public class eee extends OpMode {
    private PIDController controller;
    public static double p = 0.0075, i = 0.0085, d = 0.000000001;
    public static double f =0.000001;
    public static int target = 11;
    private final double ticks_in_degree = 28 / 180.0;
    private DcMotorEx motor;
    private CRServo turret;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static double deadband = 3.5; // Degrees of tolerance
    public static int TARGET_ID = 24;   // Set to 20 or 24 for your goals
    public static int LIMIT_LEFT = 100;
    public static int LIMIT_RIGHT = -100;


    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor= hardwareMap.get(DcMotorEx.class,"Fl");
        turret = hardwareMap.crservo.get("t");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // Initialize Vision Portal (OpenCV Backend)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camcam"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640,480))
                .build();

    }
    @Override
    public void loop() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        boolean targetFound = false;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == TARGET_ID || detection.id == 24) {
                targetFound = true;


                int target = (int) detection.ftcPose.x;







                controller.setPID(p, i, d);
                int pos = motor.getCurrentPosition();
                double pid = controller.calculate(pos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                controller.setTolerance(0.5);
                controller.atSetPoint();
                double power = pid + ff;
                turret.setPower(power);
//                if(detection.ftcPose != null){
//                    telemetry.addData("x",tag.ftcPose.x);
//                    telemetry.update();
//                    while(tag.ftcPose.x > 10 ){
//                        turret.setPower(1);
//                    }
//                    while(tag.ftcPose.x < 0){
//                        turret.setPower(-1);
//                    }
//                }

                telemetry.addData("pos", pos);
                telemetry.addData("target", target);
                telemetry.update();




            }
        }
    }
}