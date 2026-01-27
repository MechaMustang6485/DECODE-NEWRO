package org.firstinspires.ftc.teamcode.NEWRO.subsystem;




import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;

public class    RevolverRR {
    private final DcMotorEx revolver;
    private final TouchSensor touch;
    private int SetTarget;

    public RevolverRR(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        touch = hardwareMap.get(TouchSensor.class, "touchSensor");


        // reset once
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action UpdatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pwr = PIDClassForAuto.returnRevPID(SetTarget, revolver.getCurrentPosition());
                revolver.setPower(pwr);
                return true; // keep running continuously
            }
        };
    }


    public class SetTarget implements Action {
        int set;
        public SetTarget(int position){set = position;}
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SetTarget = set;
                return false; // done immediately
            }
        }
        public Action SetTarget(int pos){return new SetTarget(pos);}

    public class touch implements Action {
        int set;
        public touch(int position){set = position;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
          if ((touch.getValue() >= 0.1)){
              SetTarget = set;
          }
            return false; // done immediately
        }
    }
    public Action touch(int pos){return new touch(pos);}
    }

