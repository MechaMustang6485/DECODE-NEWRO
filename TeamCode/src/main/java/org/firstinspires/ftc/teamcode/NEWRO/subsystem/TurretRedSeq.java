package org.firstinspires.ftc.teamcode.NEWRO.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class TurretRedSeq {
    private CRServo turretServo;
    private Limelight3A limelight;
    private DcMotor Intake;

    public static int TARGET_ID = 24;
    public static double Lp = 0.02;
    public static double Ld = 0.002;
    public static double MaxPower = 0.5;
    public static double MinPower = 0.05;
    public static double Tolerance = 0.5;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    public static boolean Limits = true;
    public static int MinPo = -11000;
    public static int Maxpo = 4800;

    public TurretRedSeq(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretServo.setDirection(CRServo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(8);
        limelight.start();
        pidTimer.reset();


        Intake = hardwareMap.get(DcMotor.class, "intake");
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public class trakingTurret implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            LLResult llResult = limelight.getLatestResult();


            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 24) {

                        double TX = fr.getTargetXDegrees();
                        double power = calculatePID(TX);

                        int currentPos = Intake.getCurrentPosition();//the encoder make thing more accuret
                        if (Limits) {
                            if (currentPos >= Maxpo && power > 0) {
                                power = 0;
                            } else if (currentPos <= MinPo && power < 0) {
                                power = 0;
                            }

                        }


                        turretServo.setPower(power);

                    } else {
                        turretServo.setPower(0);


                    }
                }



            }
            return true;
        }

    }

    public Action track(){
        return new trakingTurret();
    }

    private double calculatePID(double error) {
        double deltaTime = pidTimer.seconds();
        if (deltaTime == 0) deltaTime = 0.02;
        pidTimer.reset();

        if (Math.abs(error) < Tolerance) {
            lastError = 0;
            return 0;
        }

        double P = Lp * error;
        double D = Ld * (error - lastError) / deltaTime;
        lastError = error;

        double output = P + D;

        // these are pid constrant
        if (Math.abs(output) < MinPower) {
            output = Math.signum(output) * MinPower;
        }
        return Math.max(-MaxPower, Math.min(MaxPower, output));
    }
}


