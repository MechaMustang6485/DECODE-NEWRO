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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter4;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchSensorRR;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Turret;

@Disabled
@Config
@Autonomous//working
public final class CloseRUnSequence extends LinearOpMode {
    public static double first = 1.3;
    public static double second = 2.5;
    public static int line = -30;

    public double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 1225;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;






    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TouchRev3 seq = new TouchRev3(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        DcMotorEx shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        Shooter4 shooter = new Shooter4(hardwareMap);
        Turret turret = new Turret(hardwareMap);



        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))

                .stopAndAdd(shooter.setVelo(HighVelocityShot))
                .strafeToLinearHeading(new Vector2d(-61.73, -16.7), Math.toRadians(0), (pose2dDual, posePath, v) -> 82)
                .stopAndAdd(seq.setTarget(48))
                .waitSeconds(0.3)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(seq.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(seq.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(seq.setTarget(0))
                .strafeToLinearHeading(new Vector2d(-58.2, -14.3), Math.toRadians(0), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(1.3)
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
                .stopAndAdd(seq.setTarget(48))
                .waitSeconds(0.3)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(seq.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(seq.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .strafeToLinearHeading(new Vector2d(-83.3, -51.1), Math.toRadians(-42.9), (pose2dDual, posePath, v) -> 90)

                ;



        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        seq.updatePID(),
                        seq.updateTouchAdvance(),
                        shooter.new UpdateShooter(),
                        turret.track()
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

    public static class armAction implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        Servo arm;
        double armPos;

        public armAction(Servo s, double position) {
            this.arm = s;
            this.armPos = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                arm.setPosition(armPos);
                initialized = true;
            }

            return timer.seconds() < 0.1;//don't touch
        }

    }
}








