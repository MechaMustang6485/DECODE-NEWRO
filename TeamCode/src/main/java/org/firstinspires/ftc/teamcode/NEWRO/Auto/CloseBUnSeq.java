package org.firstinspires.ftc.teamcode.NEWRO.Auto;

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter4;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;


@Config
@Autonomous//working
public final class CloseBUnSeq extends LinearOpMode {
    public static double first = 1.3;
    public static double second = 2.5;
    public static int line = -30;


    public  double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 1500;//3120
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    public void initHardware() {
        initArmOne();
        shooter();
        initTurret();
    }

    public void initArmOne() {
        Servo arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        double arminit = 0;
        arm.setPosition(arminit);

    }

    public void initTurret(){
        CRServo turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void shooter() {
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
        CRServo turret = hardwareMap.get(CRServo.class, "Turret");
        Shooter3 shooter = new Shooter3(hardwareMap);
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        TouchRev2 Revolver = new TouchRev2(hardwareMap);


        initHardware();
        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))

                .stopAndAdd(shooter.setVelo(HighVelocityShot))
                .strafeToLinearHeading(new Vector2d(-54.1, 11.38), Math.toRadians(0), (pose2dDual, posePath, v) -> 82)
                //shooting #1
                // .waitSeconds(10)
                .stopAndAdd(new BackB6.Turret(turret,0.000001))
                .stopAndAdd(shooter.setVelo(HighVelocityShot))
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.3)
                .stopAndAdd(new BackB6.armAction(arm, 0.3))
                .stopAndAdd(new BackB6.armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new BackB6.armAction(arm, 0.3))
                .stopAndAdd(new BackB6.armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new BackB6.armAction(arm, 0.3))
                .stopAndAdd(new BackB6.armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(0))
                .stopAndAdd(new BackB6.Intake(intake, 1))
                .strafeToLinearHeading(new Vector2d(-44.32, 36), Math.toRadians(38),(pose2dDual, posePath, v) -> 82)
                .waitSeconds(1.3)

                .afterTime(first,seq.setTarget(96))
                .afterTime(second,seq.setTarget(192))
                .lineToX(-42)
                .waitSeconds(0.3)
                .lineToX(-40)
                .waitSeconds(0.2)
                .lineToX(-30)

                .afterTime(first, seq.setTarget(96))
                .afterTime(second, seq.setTarget(192))
                .strafeToLinearHeading(new Vector2d(-52, -36), Math.toRadians(-40), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(-35.9, -52.0), Math.toRadians(-40), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d( -55.2, -12.0), Math.toRadians(10.4), (pose2dDual, posePath, v) -> 90)
                .stopAndAdd(Revolver.setTarget(0))
                .waitSeconds(0.3)
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.5)
                .stopAndAdd(new BackB6.armAction(arm, 0.3))
                .stopAndAdd(new BackB6.armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new BackB6.armAction(arm, 0.3))
                .stopAndAdd(new BackB6.armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new BackB6.armAction(arm, 0.3))
                .stopAndAdd(new BackB6.armAction(arm, 0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-83.3, -51.1), Math.toRadians(-42.9), (pose2dDual, posePath, v) -> 90)
                .stopAndAdd(shooter.setVelo(LowVelocityShot))
                ;



        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        Revolver.disableSensor(),
                        Revolver.updatePID(), // Always running background PID
                        shooter.new UpdateShooter()
                )
        )
        ;


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

    public static class Intake implements Action {
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
    public static class Turret implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        CRServo turret;
        double power;

        public Turret(CRServo s, double power) {
            this.turret = s;
            this.power = power;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                turret.setPower(power);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }
    }

}
