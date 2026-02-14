package org.firstinspires.ftc.teamcode.NewRo2.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.DistanceProcessor;

@TeleOp
public class DistanceTest extends OpMode {

    DistanceProcessor Distance = new DistanceProcessor();

    @Override
    public void init() {
        Distance.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Distance From Ball: ", Distance.getDistance());
        if (Distance.getDistance() <= 3) {
            telemetry.addLine("Hi");
            telemetry.update();
        }
    }
}
