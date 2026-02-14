package org.firstinspires.ftc.teamcode.NewRo2.subsystem;




import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForTele;

@Config
public class T {

    // --- Configurable Constants ---
    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = 288;
    public static int AT_TARGET_TOL = 8;
    public static double MOTOR_POWER_LIMIT = 0.6;
    public static double distance = 1;

    private DcMotorEx Intake;

    // --- Hardware ---
    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;
    private final NormalizedColorSensor color1;

    // --- State Variables ---
    private int targetPosition = 0;
    private int ballCount = 0;
    private boolean lastTouchState = false;
    private boolean autoLoadingEnabled = true;

    public enum BallSlot {
        EMPTY,

        FULL,

        PURPLE,

        GREEN,

        SHOOTING,

        BUMPING
    }

    public T(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        color1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        color1.setGain(12);

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
        // We clip it to a slightly higher max or remove the clip for the shoot sequence
        this.targetPosition = ticks;
    }

    public boolean ballinslot1(double threshold) {
        double distance = ((DistanceSensor) color1).getDistance(DistanceUnit.CM);
        return (distance < threshold);
    }
/*
    public BallSlot ballColor(NormalizedColorSensor color) {
        float[] hsv = new float[3];
        Color.colorToHSV(color.getNormalizedColors().toColor(), hsv);
        if (hsv[0] >= 100 && hsv[0] <= 180) {
            return BallSlot.GREEN;
        } else if (hsv[0] >= 181 && hsv[0] <= 255) {
            return BallSlot.PURPLE;
        }
        return BallSlot.FULL;
    }

 */

    private void handleAutoLoad() {
        boolean isPressed = touchSensor.isPressed();


        // Rising edge detection (checks if button was JUST pressed)
        if (autoLoadingEnabled && isPressed && !lastTouchState) {
            if (ballCount < MAX_SLOTS) {
                ballCount++;
                goToSlot(ballCount);
            }
        }


        if (autoLoadingEnabled && !isPressed && !lastTouchState && ballinslot1(distance) ){
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

        // Safety clip
        power = Range.clip(power, -MOTOR_POWER_LIMIT, MOTOR_POWER_LIMIT);

        revolver.setPower(power);
    }

    // --- Manual Controls ---

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
