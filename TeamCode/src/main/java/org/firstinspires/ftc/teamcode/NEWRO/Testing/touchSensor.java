package org.firstinspires.ftc.teamcode.NEWRO.Testing;






import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@Config//important
@TeleOp
public class touchSensor extends OpMode {
    private PIDFController controller;//important

    public static double p = 0.1, i = 0, d= 0.0002;
    public static double f = 0.0001 ;

    private static final int home = 0;

    public static int target = home;//this number can be used for the shoot position.

    public static int intake = 96;

    public static int shoot = 48;

    public boolean FULL = true;


    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    private DcMotorEx Revolver;
    private TouchSensor touch;
    private DcMotorEx Intake;



    @Override
    public void init(){
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//allow to do stuff in dash board
        Revolver = hardwareMap.get(DcMotorEx.class,"revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);//better stopping
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);//it better to run without encoders because it is faster
        final int home = 0;

        touch = hardwareMap.get(TouchSensor.class, "touchSensor");

        Intake = hardwareMap.get(DcMotorEx.class, "intake");
    }


    @Override
    public void loop(){



        controller.setPIDF(p, i, d, f);
        int revpose = Revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);//math
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;//math

        controller.setTolerance(0.5);//makes more accurete
        controller.atSetPoint();//this always paired with setTolerance
        double power = pid + ff;//math that sets the power
        Revolver.setPower(power);
        telemetry.addData("pose1",revpose);

        boolean pressed = (touch.getValue() >= 0.1);
        if (FULL) {
            if (pressed) {
                if (target == 0) {
                    target = intake;
                } else if (target == 96) {
                    target = 192;
                } else {
                    target = home;
                }
                FULL = false;
            }
        }

        if (gamepad1.yWasPressed()) {
            if (target == 0) {
                target = 144;
            } else if (target == 144) {
                target = 240;
            }else {
                target = shoot;
                FULL = true;
            }
        }

        if (gamepad1.dpadLeftWasPressed()) Intake.setPower(1);
        if (gamepad1.dpadRightWasPressed())Intake.setPower(0);





/*
        if (gamepad1.dpadUpWasPressed()) {
            intake.setPower(1);
        }

        if (gamepad1.dpadUpWasPressed()) {
            intake.setPower(0);
        }

 */
        updateTelemetry();


    }




    private void updateTelemetry() {

        telemetry.addData("Target",target);
        telemetry.update();
    }
}


