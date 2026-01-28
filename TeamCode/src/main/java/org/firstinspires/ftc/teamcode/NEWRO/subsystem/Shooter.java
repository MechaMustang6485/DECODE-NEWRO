package org.firstinspires.ftc.teamcode.NEWRO.subsystem;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Shooter {

    // Hardware
    private DcMotorEx shooterT, shooterB;

    // Tuning Constants (Static so Dashboard sees them)
    public static double p = 3, i = 0, d = 0;
    public static double f = 3.6;

    // Default Velocity targets
    public static double HIGH_VELOCITY = 5000;
    public static double LOW_VELOCITY = 4000;

    public Shooter(HardwareMap hardwareMap) {
        // Initialize motors using your specific mapping
        // Note: Preserving your swap (shooterB variable -> "shooterT" config)
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");

        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- ACTIONS ---

    /**
     * Action to spin up the shooter to a specific velocity.
     * Ends immediately (does not wait for spinup), letting the auto continue.
     */
    public Action spinUp(double velocity) {
        return new SpinUpAction(velocity);
    }

    /**
     * Action to stop the shooter.
     */
    public Action stop() {
        return new SpinUpAction(0);
    }

    /**
     * Inner class that handles the actual hardware command
     */
    private class SpinUpAction implements Action {
        private boolean initialized = false;
        private double targetVel;

        public SpinUpAction(double targetVel) {
            this.targetVel = targetVel;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Apply PIDF Coefficients every time we send a command
                // This ensures Dashboard tuning updates work instantly
                PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);

                shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
                shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

                shooterB.setVelocity(targetVel);
                shooterT.setVelocity(targetVel);

                initialized = true;
            }
            // Return false immediately so the Action finishes and Auto moves to the next step
            return false;
        }
    }
}