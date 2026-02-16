package org.firstinspires.ftc.teamcode.NewRo2.AutoTest;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;


@Config
@Autonomous//working
public final class CloseRSeq extends LinearOpMode {
    public static double first = 1.3;
    public static double second = 2.5;
    public static int line = -30;

    public double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 1500;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    public static double mover = 20;

    public void initHardware() {
        intitShooter();
    }

    public void intitShooter() {
        DcMotorEx shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TouchRev3 seq = new TouchRev3(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        DcMotorEx shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        Shooter3 shooter = new Shooter3(hardwareMap);


        initHardware();
        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))

                .stopAndAdd(shooter.setVelo(HighVelocityShot))
                .strafeToLinearHeading(new Vector2d(-61.73, -16.7), Math.toRadians(62.7), (pose2dDual, posePath, v) -> 82)
                .stopAndAdd(seq.scanLimelightPattern())
                .strafeToLinearHeading(new Vector2d(-58.2, -14.3), Math.toRadians(18.7), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(1.3)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .stopAndAdd(seq.setTarget(0))
                .stopAndAdd(new Intake(intake, 1))
                .strafeToLinearHeading(new Vector2d(-63.067, -25.564), Math.toRadians(-40), (pose2dDual, posePath, v) -> 82)

             //   .afterTime(first, seq.setTarget(96))
              //  .afterTime(second, seq.setTarget(192))
                .stopAndAdd(seq.resetTouch())
                .strafeToLinearHeading(new Vector2d(-52, -36), Math.toRadians(-40), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(-35.9, -52.0), Math.toRadians(-40), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d( -55.2, -12.0), Math.toRadians(10.4), (pose2dDual, posePath, v) -> 90)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .strafeToLinearHeading(new Vector2d(-83.3, -51.1), Math.toRadians(-42.9), (pose2dDual, posePath, v) -> 90)
                .stopAndAdd(new Shooter(shooterB, shooterT, 0))
                ;



        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        seq.updatePID(),
                        seq.updateTouchAdvance(),
                        shooter.new UpdateShooter()
                )
        );
    }

    public class Intake implements Action {
        private boolean initialized = false;//don't touch
        ElapsedTime timer;
        DcMotor intake;
        double power;


        public Intake(DcMotor b, double w) {
            this.intake = b;
            this.power = w;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                intake.setPower(power);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }


    }

    public class Shooter implements Action {
        private boolean initialized = false;//don't touch
        ElapsedTime timer;
        DcMotorEx shooterB;
        DcMotorEx shooterT;
        double power;


        public Shooter(DcMotorEx b, DcMotorEx t, double w) {
            this.shooterB = b;
            this.shooterT = t;
            this.power = w;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
                shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
                shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
                shooterB.setVelocity(power);
                shooterT.setVelocity(power);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }
    }
}







