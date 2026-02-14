package org.firstinspires.ftc.teamcode.NewRo2.subsystem;




import android.text.method.Touch;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;

public class TouchSensorRR {
    private final TouchSensor touch;
    private int SetTarget;
    public boolean Touched = false;

    public TouchSensorRR(HardwareMap hardwareMap) {
        touch = hardwareMap.get(TouchSensor.class, "touchSensor");

    }

    public Action UpdateTouch() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if ((touch.getValue() >= 0.1)) {
                    Touched = true;
                } else {
                    Touched = false;
                }
                return true;
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

            }
            return false; // done immediately
        }
    }
    public Action touch(int pos){return new touch(pos);}
}

