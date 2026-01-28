package org.firstinspires.ftc.teamcode.NEWRO.Auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.RevolverRR;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchSensorRR;


@Config
@Autonomous
public final class CloseBSeq extends LinearOpMode {
    public static double first = 0.5;
    public static double second = 1;
    public static int line = -30;

    public  double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 3500;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;
    public static double mover = 20;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TouchRev3 seq = new TouchRev3(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        DcMotorEx shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))

                .stopAndAdd(shooter.spinUp(Shooter.LOW_VELOCITY))
                .strafeToLinearHeading(new Vector2d(-67.86, 13.53), Math.toRadians(-51), (pose2dDual, posePath, v) -> 80)
                .stopAndAdd(seq.scanLimelightPattern())
                .strafeToLinearHeading(new Vector2d(-64.66, 14.86), Math.toRadians(-5), (pose2dDual, posePath, v) -> 80)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .stopAndAdd(seq.setTarget(0))
                .stopAndAdd(new Intake(intake, 1))
                .strafeToLinearHeading(new Vector2d(-41.32, 36), Math.toRadians(38),(pose2dDual, posePath, v) -> 42)
                .stopAndAdd(shooter.stop())
                .afterTime(first,seq.setTarget(96))
                .afterTime(second,seq.setTarget(192))
                .lineToX(-30)
                .waitSeconds(0.1)
                .lineToX(-20)
                .stopAndAdd(shooter.spinUp(Shooter.LOW_VELOCITY))
                .strafeToLinearHeading(new Vector2d(-64.89, 15.29), Math.toRadians(-7), (pose2dDual, posePath, v) -> 80)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .stopAndAdd(shooter.stop())
                .stopAndAdd(seq.setTarget(0))
                .stopAndAdd(new Intake(intake, 1))
                .strafeToLinearHeading(new Vector2d(-62.19, 63.36), Math.toRadians(39), (pose2dDual, posePath, v) -> 80)
                .afterTime(first,seq.setTarget(96))
                .afterTime(second,seq.setTarget(192))
                .strafeToLinearHeading(new Vector2d(-62.19, 33.36), Math.toRadians(39), (pose2dDual, posePath, v) -> 80)
                .stopAndAdd(new Intake(intake, 0))
                .stopAndAdd(shooter.spinUp(Shooter.LOW_VELOCITY))
                .strafeToLinearHeading(new Vector2d(-65.33, 14.90), Math.toRadians(-6), (pose2dDual, posePath, v) -> 80)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .strafeToLinearHeading(new Vector2d(-64.16, 34.42), Math.toRadians(-5), (pose2dDual, posePath, v) -> 80)


                ;


        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        seq.updatePID()
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
}







