package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class bobobobobobobobobobobobobo extends LinearOpMode {

    public Servo arm;

    public double Pos;

    public void initHardware() {
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()
        ) {
            if (gamepad1.dpadUpWasPressed()) {
                Pos = Pos + 0.1;
            }

            if (gamepad1.dpadDownWasPressed()) {
                Pos = Pos - 0.1;
            }

            arm.setPosition(Pos);

            telemetry.addData("CurrentPos", Pos);
            telemetry.update();
        }
    }
}
