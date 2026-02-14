package org.firstinspires.ftc.teamcode.NEWRO.Processors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceProcessor {
    private DistanceSensor color1;

    public void init(HardwareMap hwMap) {
        color1 = hwMap.get(DistanceSensor.class, "color1");

    }

    public double getDistance() {
        return color1.getDistance(DistanceUnit.CM);
    }
}
