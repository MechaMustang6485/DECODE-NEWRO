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
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchSensorRR;

@Disabled
@Config
@Autonomous
public final class CloseB extends LinearOpMode {

    public  double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 3500;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    public void initHardware() {
        initArmOne();
        shooter();
    }

    public void initArmOne() {
        Servo arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        double arminit = 0;
        arm.setPosition(arminit);

    }

    public void shooter(){


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
        RevolverRR revolver = new RevolverRR(hardwareMap);
        TouchRev2 Revolver = new TouchRev2(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        DcMotorEx shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");





        initHardware();
        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))

                .stopAndAdd(new Shooter(shooterB,shooterT,curTargetVelocity))
                .strafeToLinearHeading(new Vector2d(-59.87, 8.1), Math.toRadians(0), (pose2dDual, posePath, v) -> 42)
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.3)
                .stopAndAdd(new armAction(arm,0.3))
                .stopAndAdd(new armAction(arm,0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm,0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))

                .strafeToLinearHeading(new Vector2d(-46.23, 31.95), Math.toRadians(41.04), (pose2dDual, posePath, v) -> 42)
                .stopAndAdd(Revolver.setTarget(0))
                .stopAndAdd(new Shooter(shooterB,shooterT,0))
                .stopAndAdd(new Intake(intake,1))
                .lineToX(-30)
                .waitSeconds(0.5)
                .lineToX(-20)
                .waitSeconds(0.5)
                .stopAndAdd(new Intake(intake, 0))

                .stopAndAdd(new Shooter(shooterB,shooterT,curTargetVelocity))
                .strafeToLinearHeading(new Vector2d(-63.45, 15.16), Math.toRadians(-6.83), (pose2dDual, posePath, v) -> 42)
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(2)
                .stopAndAdd(new armAction(arm,0.3))
                .stopAndAdd(new armAction(arm,0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm,0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .strafeToLinearHeading(new Vector2d(-63.45, 15.16), Math.toRadians(-60), (pose2dDual, posePath, v) -> 42)



                /*
                .stopAndAdd(new armAction(arm,0))
                .waitSeconds(0.2)
                .stopAndAdd(revolver.SetTarget(144))
                .stopAndAdd(new armAction(arm,0.3))


                 */


                ;


        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        Revolver.disableSensor(),
                        Revolver.updatePID(), // Always running background PID

                        new SequentialAction(
                                new SleepAction(8),
                        Revolver.enableSensor(),
                                new SleepAction(9),
                                Revolver.resetRevolver(),
                                Revolver.disableSensor()

                        )
        )
        );
}
    public class armAction implements Action {
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







