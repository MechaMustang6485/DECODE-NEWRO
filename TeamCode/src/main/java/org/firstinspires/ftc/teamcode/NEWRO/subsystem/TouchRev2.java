package org.firstinspires.ftc.teamcode.NEWRO.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;

public class TouchRev2 {
    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;

    // --- CONFIGURATION ---
    private final int TICKS_PER_SLOT = 96;
    private final int MAX_SLOTS = 3;
    // 288 ticks is the absolute max (3 * 96)
    private final int MAX_POSITION = MAX_SLOTS * TICKS_PER_SLOT;

    private int targetPosition = 0;
    private int currentSlotCount = 0;

    private boolean lastButtonState = false;
    private boolean isSensorEnabled = false;

    public TouchRev2(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset hardware encoder to 0 on init
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action updatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // 1. Sensor Logic
                if (isSensorEnabled) {
                    boolean isPressed = touchSensor.isPressed();

                    // Logic: Button pressed + NOT pressed before + NOT full yet
                    if (isPressed && !lastButtonState && currentSlotCount < MAX_SLOTS) {
                        currentSlotCount++;
                        targetPosition = currentSlotCount * TICKS_PER_SLOT;
                    }
                    lastButtonState = isPressed;
                }

                // 2. PID Control
                double pwr = PIDClassForAuto.returnRevPID(targetPosition, revolver.getCurrentPosition());
                revolver.setPower(pwr);

                // --- Telemetry ---
                packet.put("Rev Sensor Active", isSensorEnabled);
                packet.put("Rev Slot", currentSlotCount);
                packet.put("Rev Target", targetPosition);
                packet.put("Rev Actual", revolver.getCurrentPosition());

                return true;
            }
        };
    }

    // --- ACTIONS ---

    /**
     * Call this after scoring!
     * It moves the target back to 0 so you can pick up 3 new balls.
     */
    public Action resetRevolver() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = 0;
                currentSlotCount = 0;
                return false; // Done immediately
            }
        };
    }

    public Action enableSensor() {
        return packet -> {
            isSensorEnabled = true;
            return false;
        };
    }

    public Action disableSensor() {
        return packet -> {
            isSensorEnabled = false;
            return false;
        };
    }

    /**
     * DANGEROUS: Only use this if the hardware encoder drift is bad
     * and you are physically at the "Zero" position.
     */
    public Action resetHardwareEncoder() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPosition = 0;
                currentSlotCount = 0;
                return false;
            }
        };
    }

    public Action setTarget(int pos) {
        return new SetTargetAction(pos);
    }

    public class SetTargetAction implements Action {
        int pos;
        public SetTargetAction(int position){ this.pos = position; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Safety Clamp: Don't let target go above 288 (MAX_POSITION)
            if (pos > MAX_POSITION) {
                targetPosition = MAX_POSITION;
            } else {
                targetPosition = pos;
            }

            // Sync the slot count variable for logic consistency
            currentSlotCount = Math.round((float) targetPosition / TICKS_PER_SLOT);
            return false;
        }
    }
}