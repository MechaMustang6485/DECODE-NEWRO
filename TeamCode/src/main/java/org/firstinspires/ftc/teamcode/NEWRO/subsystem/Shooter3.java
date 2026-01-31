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
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Shooter3 {
    private final DcMotorEx shooterB, shooterT;
    private final VoltageSensor batteryVoltage;

    // TUNE THESE IN DASHBOARD
    public static double P = 13, I = 0, D = 0;
    public static double F = 11.3;
    public static double activeTargetVelo = 0;

    public Shooter3(HardwareMap hardwareMap) {
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public Action setVelo(double velocity) {
        return packet -> {
            activeTargetVelo = velocity;
            return false;
        };
    }


    public class UpdateShooter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double voltage = batteryVoltage.getVoltage();
            double voltageComp = 12.0 / Math.max(voltage, 1.0);

            PIDFCoefficients coeffs = new PIDFCoefficients(P, I, D, F * voltageComp);
            shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
            shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

            shooterB.setVelocity(activeTargetVelo);
            shooterT.setVelocity(activeTargetVelo);

            packet.put("Shooter Target", activeTargetVelo);
            packet.put("Shooter Actual", shooterB.getVelocity());
            return true;
        }
    }
}

