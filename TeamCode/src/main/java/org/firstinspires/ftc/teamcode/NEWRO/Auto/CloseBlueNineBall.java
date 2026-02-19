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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter4;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev4;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Turret;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Turret4;


@Config
@Autonomous
public final class CloseBlueNineBall extends LinearOpMode {

    public static int first = 1;
    public static int second = 2;

    public  double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 1250;//3120
    public double LowVelocityShot = 0;
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
       DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
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
        TouchRev4 Revolver = new TouchRev4(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        Shooter4 shooter = new Shooter4(hardwareMap);
        Turret turret = new Turret(hardwareMap);



        initHardware();
        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))
                //shooting #1
                // .waitSeconds(10)
                .stopAndAdd(shooter.setVelo(HighVelocityShot))
                .strafeToLinearHeading(new Vector2d(-60.46, 15.57), Math.toRadians(1.89), (pose2dDual, posePath, v) -> 82)
                .waitSeconds(0.1)
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.3)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(0))
                .stopAndAdd(new Intake(intake, -1))
                //spike line #1
                .stopAndAdd(Revolver.resetTouch())
                .strafeToLinearHeading(new Vector2d(-48.4, 29), Math.toRadians(40),(pose2dDual, posePath, v) -> 82)
                .strafeToLinearHeading(new Vector2d(-20.13, 53.33), Math.toRadians(40.0), (pose2dDual, posePath, v) -> 30)
                .strafeToLinearHeading(new Vector2d(-59.1, 15.41), Math.toRadians(7.58), (pose2dDual, posePath, v) -> 80)
                .waitSeconds(0.3)
                //shooting #2
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .waitSeconds(0.3)
                .stopAndAdd(Revolver.setTarget(0))
                .stopAndAdd(Revolver.resetTouch())
                .stopAndAdd(new Back6test.Intake(intake, -1))
                .strafeToLinearHeading(new Vector2d(-64.59, 58.95), Math.toRadians(40), (pose2dDual, posePath, v) -> 80)
                .strafeToLinearHeading(new Vector2d(-35.02, 85.57), Math.toRadians(40), (pose2dDual, posePath, v) -> 20)
                .strafeToLinearHeading(new Vector2d(-49.7, 73.37), Math.toRadians(40), (pose2dDual, posePath, v) -> 80)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-68.55, 15.59), Math.toRadians(-0.032), (pose2dDual, posePath, v) -> 80)
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .waitSeconds(1)
                .stopAndAdd(Revolver.setTarget(0))
                .strafeToLinearHeading(new Vector2d(-49.32,46.17), Math.toRadians(40), (pose2dDual, posePath, v) -> 80)

                //spike line #2
                /*
                .stopAndAdd(new Intake(intake, 1))
                .stopAndAdd(Revolver.setTarget(0))
                .strafeToLinearHeading(new Vector2d(74, -16), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(74, -7.1), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                .waitSeconds(0.3)
                .stopAndAdd(Revolver.setTarget(96))
                .lineToX(79)
                .strafeToLinearHeading(new Vector2d(74, -1), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                .waitSeconds(0.3)
                .stopAndAdd(Revolver.setTarget(192))
                .lineToX(85)
                .waitSeconds(0.5)
                .lineToX(35)
                //shooting #3
                .stopAndAdd(Revolver.setTarget(0))
                .waitSeconds(0.3)
                .stopAndAdd(Revolver.setTarget(48))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(144))
                .waitSeconds(0.7)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .stopAndAdd(Revolver.setTarget(240))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, 0.3))
                .stopAndAdd(new armAction(arm, 0))
                .waitSeconds(1)
                 */
                ;







                /*.strafeToLinearHeading(new Vector2d(26.2, -65.3), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                .strafeToLinearHeading(new Vector2d(70, -65.3), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                .strafeToLinearHeading(new Vector2d(-2, -4), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                .strafeToLinearHeading(new Vector2d(14, -8.5), Math.toRadians(0), (pose2dDual, posePath, v) -> 80);

                 */


        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build(),
                        Revolver.updatePID(), // Always running background PID
                        Revolver.updateTouchAdvance(),
                        shooter.new UpdateShooter(),
                        turret.track()

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


}
