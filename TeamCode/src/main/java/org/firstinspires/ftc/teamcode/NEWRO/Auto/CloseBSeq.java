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
    public static int first = 1;
    public static int second = 2;
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
        RevolverRR revolver = new RevolverRR(hardwareMap);
        TouchRev3 seq = new TouchRev3(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        DcMotorEx shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        TouchSensorRR touch = new TouchSensorRR(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        TouchRev2 Revolver = new TouchRev2(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))

                .stopAndAdd(shooter.spinUp(Shooter.LOW_VELOCITY))
                .strafeToLinearHeading(new Vector2d(-62.42, 5.5), Math.toRadians(-50), (pose2dDual, posePath, v) -> 42)
                .stopAndAdd(seq.scanLimelightPattern())
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-51.68, 12.6), Math.toRadians(-7), (pose2dDual, posePath, v) -> 42)
                .stopAndAdd(seq.runSequence())
                //.stopAndAdd(new Intake(intake, 1))
                //.strafeToLinearHeading(new Vector2d(-46.23, 31.95), Math.toRadians(41.04), (pose2dDual, posePath, v) -> 42)
                //.stopAndAdd(Revolver.setTarget(96))
                //.afterTime(second,Revolver.setTarget(192))
                //.lineToX(line)
                //.strafeToLinearHeading(new Vector2d(-63.45, 15.16), Math.toRadians(-6.83), (pose2dDual, posePath, v) -> 42)
                //.strafeToLinearHeading(new Vector2d(-63.45, 15.16), Math.toRadians(-60), (pose2dDual, posePath, v) -> 42)
                //.stopAndAdd(new Intake(intake, 0))
                //.stopAndAdd(seq.runSequence())

                //.strafeToLinearHeading(new Vector2d(-63.45, 15.16), Math.toRadians(-60), (pose2dDual, posePath, v) -> 42)



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







