package org.firstinspires.ftc.teamcode.NewRo2.Testing;




import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config//important
@Disabled
@TeleOp
public class bob extends OpMode {
    private PIDFController controller;//important

    public static double prev = 0.1, irev = 0, drev = 0.000001;
    public static double frev = 0.000001;

    public static int target = 96;

    public static double pshot = 2.3013, ishot = 0, dshot = 0;
    public static double fshot = 14;

    public double HighVelocityShot = 2000;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    private DcMotorEx intake;

    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    private DcMotorEx Revolver;

    private TouchSensor touch;

    public int BallCount;

    public boolean BallFull = false;

    public boolean waiting = false;

    private boolean wasPressed = false;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){
        controller = new PIDController(prev, irev, drev);

        touch = hardwareMap.get(TouchSensor.class, "touch");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//allow to do stuff in dash board
        Revolver = hardwareMap.get(DcMotorEx.class,"revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);//better stopping
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);//it better to run without encoders because it is faster

    }

    @Override
    public void loop(){
        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

        controller.setPIDF(prev, irev, drev, frev);
        int revpose = Revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);//math
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * frev; // math

        controller.setTolerance(0.5);//makes more accurete
        controller.atSetPoint();//this always paired with setTolerance
        double power = pid + ff;//math that sets the power
        Revolver.setPower(power);
        telemetry.addData("pose1",revpose);


        if (gamepad1.dpadUpWasPressed()) {
            intake.setPower(1);
        }

        if (gamepad1.dpadUpWasPressed()) {
            intake.setPower(0);
        }

        if (gamepad1.aWasReleased()){
            Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    // BallFull logic
        if (BallCount == 3) {
            BallFull = true;
            intake.setVelocity(0);
        }

        // Touch sensor increments BallCount
        boolean pressed = (touch.getValue() >= 0.1);
        if (pressed && !wasPressed && !BallFull) {
            runtime.reset();
            waiting = true;
            BallCount = BallCount + 1;
            Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (waiting) {
            if (runtime.seconds() >= 1) {
                Revolver.setTargetPosition(target);
                waiting = false;
            }
        }
        wasPressed = pressed;


        telemetry.addData("Target",target);
        telemetry.update();

    }
}


