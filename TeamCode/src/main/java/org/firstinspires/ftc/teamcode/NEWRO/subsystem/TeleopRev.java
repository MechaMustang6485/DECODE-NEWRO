package org.firstinspires.ftc.teamcode.NEWRO.subsystem;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;
import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForTele;

@Config
public class TeleopRev {


    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = 288;
    public static double MOTOR_POWER_LIMIT = 0.6;

    private DcMotorEx Intake;


    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;


    private int targetPosition = 0;
    private int ballCount = 0;
    private boolean lastTouchState = false;
    private boolean autoLoadingEnabled = true;

    public TeleopRev(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    /**
     * Call this every loop in your Teleop OpMode.
     */
    public void update() {
        handleAutoLoad();
        handleRevolverPID();
    }

    public void setTargetPosition(int ticks) {

        this.targetPosition = ticks;
    }

    private void handleAutoLoad() {
        boolean isPressed = touchSensor.isPressed();

        // Rising edge detection (checks if button was JUST pressed)
        if (autoLoadingEnabled && isPressed && !lastTouchState) {
            if (ballCount < MAX_SLOTS) {
                ballCount++;
                goToSlot(ballCount);
            }
        }

        lastTouchState = isPressed;
    }
    public void IntakePower(double power){
        Intake.setPower(power);
    }

    private void handleRevolverPID() {

        double power = PIDClassForTele.returnRevPID(targetPosition, revolver.getCurrentPosition());


        power = Range.clip(power, -MOTOR_POWER_LIMIT, MOTOR_POWER_LIMIT);

      revolver.setPower(power);
    }



    /**
     * Set target based on slot index (0, 1, 2, or 3)
     */
    public void goToSlot(int slot) {
        targetPosition = Range.clip(slot * TICKS_PER_SLOT, 0, MAX_POSITION);
        ballCount = slot;
    }

    /**
     * Resets encoder and target back to zero
     */
    public void resetRevolver() {
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
        ballCount = 0;
    }

    public void setAutoLoading(boolean enabled) {
        this.autoLoadingEnabled = enabled;
    }

    public void setRawTarget(int ticks) {
        this.targetPosition = ticks;
        // We update ballCount to the closest slot so auto-loading doesn't get confused
        this.ballCount = Math.round((float)ticks / TICKS_PER_SLOT);
    }

    // --- Getters for Telemetry ---
    public int getBallCount() { return ballCount; }
    public int getEncoder() { return revolver.getCurrentPosition(); }
    public int getTarget() { return targetPosition; }
    public boolean isTouchPressed() { return touchSensor.isPressed(); }
}
