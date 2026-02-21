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
public class TeleopRev1 {

    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = 288;
    public static double MOTOR_POWER_LIMIT = 1.0;

    // NEW: Multiplier to make the manual movement slow and controlled
    public static double MANUAL_SPEED_MULTIPLIER = 0.3;

    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;

    private int targetPosition = 0;
    private int ballCount = 0;
    private boolean lastTouchState = false;
    private boolean autoLoadingEnabled = true;

    // NEW: Variable to store manual input
    private double manualPower = 0.0;

    public TeleopRev1(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


    public void setManualPower(double power) {
        this.manualPower = power;
    }

    private void handleAutoLoad() {
        boolean isPressed = touchSensor.isPressed();

        // Prevent auto-load from triggering if we are manually moving the revolver
        if (autoLoadingEnabled && isPressed && !lastTouchState && Math.abs(manualPower) < 0.05) {
            if (ballCount < MAX_SLOTS) {
                ballCount++;
                goToSlot(ballCount);
            }
        }

        lastTouchState = isPressed;
    }

    private void handleRevolverPID() {
        // NEW: Check if manual power is being applied (with a small deadzone)
        if (Math.abs(manualPower) > 0.05) {
            // Apply raw manual power, scaled down for slow movement
            revolver.setPower(manualPower * MANUAL_SPEED_MULTIPLIER);

            // Constantly update the target position to the current position.
            // When you let go, the PID will lock it right where it stopped.
            targetPosition = revolver.getCurrentPosition();

            // Sync the ball count to the closest slot so auto-loading doesn't get confused
            ballCount = Math.round((float)targetPosition / TICKS_PER_SLOT);
        } else {
            // Normal PID control
            double power = PIDClassForTele.returnRevPID(targetPosition, revolver.getCurrentPosition());
            power = Range.clip(power, -MOTOR_POWER_LIMIT, MOTOR_POWER_LIMIT);
            revolver.setPower(power);
        }
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

    public void overide() {
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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