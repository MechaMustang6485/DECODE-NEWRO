package org.firstinspires.ftc.teamcode.NEWRO.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;

public class TouchRev {
    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;
    public boolean STFUENCODER = false;

    // --- CONFIGURATION ---
    private final int TICKS_PER_SLOT = 96;

    private int targetPosition = 0;
    private int currentSlotCount = 0;

    // To track the button state from the previous loop
    private boolean lastButtonState = false;

    // NEW: The switch to control if the button works
    private boolean isSensorEnabled = false;

    public TouchRev(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);

        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action updatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // 1. Only check the button if we enabled it!
                if (isSensorEnabled) {
                    boolean isPressed = touchSensor.isPressed();

                    // Check for rising edge (pressed now, but wasn't before)
                    if (isPressed && !lastButtonState) {
                        currentSlotCount++; // Go to next slot
                        targetPosition = currentSlotCount * TICKS_PER_SLOT;
                    }
                    lastButtonState = isPressed;
                }

                // 2. PID always runs to hold the motor steady
                double pwr = PIDClassForAuto.returnRevPID(targetPosition, revolver.getCurrentPosition());
                revolver.setPower(pwr);

                // --- Telemetry ---
                packet.put("Sensor Enabled", isSensorEnabled);
                packet.put("Current Slot", currentSlotCount);
                packet.put("Target Pos", targetPosition);
                packet.put("Actual Pos", revolver.getCurrentPosition());

                return true; // Keeps running in the background
            }
        };
    }



    public Action enableSensor() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isSensorEnabled = true;
                return false; // Done immediately
            }
        };
    }

    public Action disableSensor() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isSensorEnabled = false;
                return false; // Done immediately
            }
        };
    }

    public Action ResetEncoder() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            targetPosition = pos;
            return false;
        }
    }
}