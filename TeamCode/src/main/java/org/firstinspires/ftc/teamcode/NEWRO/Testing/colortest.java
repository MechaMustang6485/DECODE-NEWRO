package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.ColorSensorProcessor;
@Disabled
@TeleOp
public class colortest extends LinearOpMode {
    ColorSensorProcessor colorSensorProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensorProcessor = new ColorSensorProcessor(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Slot1 Color", colorSensorProcessor.colorInSlot1().toString());
            telemetry.addData("Slot2 Color", colorSensorProcessor.colorInSlot2().toString());
            telemetry.addData("Slot3 Color", colorSensorProcessor.colorInSlot3().toString());
            telemetry.addData("Slot4 Color", colorSensorProcessor.colorInSlot4().toString());
            telemetry.addData("Slot1 Raw Color", colorSensorProcessor.rawcolorslot1());
            telemetry.addData("Slot2 Raw Color", colorSensorProcessor.rawcolorslot2());
            telemetry.addData("Slot3 Raw Color", colorSensorProcessor.rawcolorslot3());
            telemetry.addData("Slot4 Raw Color", colorSensorProcessor.rawcolorslot4());
            telemetry.update();
        }
    }
}
