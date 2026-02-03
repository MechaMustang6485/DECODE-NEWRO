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
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchSensorRR;


@Config
@Autonomous//working
public final class CloseBSeq extends LinearOpMode {
    public static double first = 0.9;
    public static double second = 2.5;

    public  double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 1500;//3120
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    public static double mover = 20;

    public void initHardware() {
        intitShooter();
    }
    public void intitShooter(){
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
               // .stopAndAdd(new Shooter(shooterB,shooterT,curTargetVelocity))
                .strafeToLinearHeading(new Vector2d(-69.73, 8.64), Math.toRadians(-44), (pose2dDual, posePath, v) -> 82)
                .stopAndAdd(seq.scanLimelightPattern())
                .strafeToLinearHeading(new Vector2d(-64.66, 14.86), Math.toRadians(-5), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(1.3)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .stopAndAdd(seq.setTarget(0))
                .stopAndAdd(new Intake(intake, 1))
                .strafeToLinearHeading(new Vector2d(-44.32, 36), Math.toRadians(38),(pose2dDual, posePath, v) -> 82)

                .afterTime(first,seq.setTarget(96))
                .afterTime(second,seq.setTarget(192))
                .lineToX(-42)
                .waitSeconds(0.3)
                .lineToX(-40)
                .waitSeconds(0.2)
                .lineToX(-30)
               // .stopAndAdd(new Shooter(shooterB,shooterT,curTargetVelocity))
                .strafeToLinearHeading(new Vector2d(-61.89, 15.29), Math.toRadians(2), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(0.2)
                .stopAndAdd(seq.runSequence3ShotsNoShooter())
                .stopAndAdd(new Shooter(shooterB, shooterT, 0))

                .stopAndAdd(seq.setTarget(0))
                .strafeToLinearHeading(new Vector2d(-74.51, 50), Math.toRadians(39), (pose2dDual, posePath, v) -> 82)
                ;
                //.stopAndAdd(new Shooter(shooterB,shooterT,curTargetVelocity))
                //.strafeToLinearHeading(new Vector2d(-65.33, 14.90), Math.toRadians(-6), (pose2dDual, posePath, v) -> 82)
                //.stopAndAdd(seq.runSequence3ShotsNoShooter())
                //.strafeToLinearHeading(new Vector2d(-64.16, 34.42), Math.toRadians(-5), (pose2dDual, posePath, v) -> 82);


        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        seq.updatePID(),
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


        public Shooter(DcMotorEx b,DcMotorEx t, double w) {
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







