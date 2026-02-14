package org.firstinspires.ftc.teamcode.NewRo2.subsystem;


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
public class Shooter2 {

    private final DcMotorEx shooterT, shooterB;


    // Tuning Constants
    public static double p = 18, i = 0.0, d = 0.0;
    public static double f = 30; // Feedforward (Tuned for ~13-14V)
    public static double VELO_TOLERANCE = 75; // Acceptable error in ticks/sec
    public static double HIGH_VELOCITY = 1200;
    public static double LOW_VELOCITY = 0;


    public Shooter2(HardwareMap hardwareMap) {
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");


        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        // Required for setVelocity to work
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        updatePIDF();
    }

    private void updatePIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public class SpinUpAction implements Action {
        private final double targetVel;
        private boolean paramsSet = false;

        public SpinUpAction(double targetVel) {
            this.targetVel = targetVel;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!paramsSet) {
                updatePIDF();
                paramsSet = true;
            }

            // Adjust target slightly based on voltage if not using internal compensation
            // If battery is low, the internal PIDF might need more 'oomph'


            shooterB.setVelocity(targetVel);
            shooterT.setVelocity(targetVel);

            double vB = shooterB.getVelocity();
            double vT = shooterT.getVelocity();

            packet.put("Target Velo", targetVel);
            packet.put("Actual Velo B", vB);
            packet.put("Actual Velo T", vT);


            if (targetVel == 0) return false; // Stop immediately if turning off

            // Return TRUE to keep running (waiting) until we are within tolerance
            return Math.abs(targetVel - vB) > VELO_TOLERANCE;
        }
    }

    public Action spinUp(double velocity) {
        return new SpinUpAction(velocity);
    }

    public Action stop() {
        return new SpinUpAction(0);
    }
}