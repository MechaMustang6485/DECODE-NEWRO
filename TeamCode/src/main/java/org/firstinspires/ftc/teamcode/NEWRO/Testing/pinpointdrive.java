package org.firstinspires.ftc.teamcode.NEWRO.Testing;





import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class pinpointdrive extends LinearOpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "Fl");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Bl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Fr");
        backRightMotor = hardwareMap.get(DcMotor.class, "Br");


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));


        telemetry.addData("Status", "Initialized. Press Start to drive field-centric.");
        telemetry.update();

        configurePinpoint();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            pinpoint.update();

            if (gamepad1.back) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                telemetry.addData("IMU Status", "Yaw Reset Initiated!");
            }



            double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);


            double y_input = -gamepad1.left_stick_y;
            double x_input = gamepad1.left_stick_x * 1.1;
            double rotation_input = gamepad1.right_stick_x;


            double rotX = x_input * Math.cos(-botHeading) - y_input * Math.sin(-botHeading);
            double rotY = x_input * Math.sin(-botHeading) + y_input * Math.cos(-botHeading);


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation_input), 1);

            double frontLeftPower = (rotY + rotX + rotation_input) / denominator;
            double backLeftPower = (rotY - rotX + rotation_input) / denominator;
            double frontRightPower = (rotY - rotX - rotation_input) / denominator;
            double backRightPower = (rotY + rotX - rotation_input) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            Pose2D pose2D = pinpoint.getPosition();

            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

    }


    public void configurePinpoint(){

        pinpoint.setOffsets(76.2, 127, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
    }

}

