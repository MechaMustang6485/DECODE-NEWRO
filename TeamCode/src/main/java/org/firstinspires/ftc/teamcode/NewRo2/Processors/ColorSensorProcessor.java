package org.firstinspires.ftc.teamcode.NewRo2.Processors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorProcessor {
    NormalizedColorSensor color1;
    NormalizedColorSensor color2;
    NormalizedColorSensor color3;
    NormalizedColorSensor color4;

    public enum BallSlot {
        EMPTY,

        FULL,

        PURPLE,

        GREEN,

        SHOOTING,

        BUMPING
    }

    public ColorSensorProcessor(HardwareMap hardwareMap) {
        color1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        color1.setGain(12);
        color2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        color2.setGain(1);
        color3 = hardwareMap.get(NormalizedColorSensor.class, "color3");
        color3.setGain(1);
        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        color4.setGain(12);
    }

    public boolean ballinslot1(double threshold) {
        double distance = ((DistanceSensor) color1).getDistance(DistanceUnit.CM);
        return (distance < threshold);
    }

    public boolean ballinslot2(double threshold) {
        double distance = ((DistanceSensor) color2).getDistance(DistanceUnit.CM);
        return (distance < threshold);
    }

    public boolean ballinslot3(double threshold) {
        double distance = ((DistanceSensor) color3).getDistance(DistanceUnit.CM);
        return (distance < threshold);
    }

    public boolean ballinslot4(double threshold) {
        double distance = ((DistanceSensor) color4).getDistance(DistanceUnit.CM);
        return (distance < threshold);
    }

    public BallSlot colorInSlot1() {
        return ballColor(color1);
    }

    public BallSlot colorInSlot2() {
        return ballColor(color2);
    }

    public BallSlot colorInSlot3() {
        return ballColor(color3);
    }

    public BallSlot colorInSlot4() { return ballColor(color4); }

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
    public float rawcolorslot1() {
        return rawBallColor(color1);
    }

    public float rawcolorslot2() {
        return rawBallColor(color2);
    }

    public float rawcolorslot3() {
        return rawBallColor(color3);
    }

    public float rawcolorslot4() {
        return rawBallColor(color4);
    }

    public float rawBallColor(NormalizedColorSensor color) {
        float[] hsv = new float[3];
        Color.colorToHSV(color.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }
}