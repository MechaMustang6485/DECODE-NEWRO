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
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Shooter5;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev3;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev32;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TouchRev33;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Turret;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.Turret2;
import org.firstinspires.ftc.teamcode.NEWRO.subsystem.TurretRedSeq;


@Config
@Autonomous
public final class CloseRedSeqTest extends LinearOpMode {

    public static int first = 1;
    public static int second = 2;

    public  double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 2.47;
    public static double HighVelocityShot = 1240;//3120
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
        TouchRev33 Revolver = new TouchRev33(hardwareMap);
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        Shooter5 shooter = new Shooter5(hardwareMap);
        TurretRedSeq turret = new TurretRedSeq(hardwareMap);



        initHardware();
        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))
                //shooting #1
                // .waitSeconds(10)
                .stopAndAdd(shooter.setVelo(HighVelocityShot))
                .strafeToLinearHeading(new Vector2d(-40, -1.4004), Math.toRadians(7), (pose2dDual, posePath, v) -> 160)
                .stopAndAdd(Revolver.scanLimelightPattern())
                .strafeToLinearHeading(new Vector2d(-66, -23), Math.toRadians(-38), (pose2dDual, posePath, v) -> 160)
                .stopAndAdd(Revolver.runSequence3ShotsNoShooter())
                .stopAndAdd(new Intake(intake, -1))
                //spike line #1
                .stopAndAdd(Revolver.resetTouch())
                .strafeToLinearHeading(new Vector2d(-36, -50), Math.toRadians(-38),(pose2dDual, posePath, v) -> 82)
                .strafeToLinearHeading(new Vector2d(-67, -22), Math.toRadians(-35), (pose2dDual, posePath, v) -> 160)
                //shooting #2
                .stopAndAdd(new Intake(intake, 0))
                .stopAndAdd(Revolver.runSequence3ShotsNoShooter())
                .stopAndAdd(Revolver.resetTouch())
                .stopAndAdd(new Intake(intake, -1))
                .strafeToLinearHeading(new Vector2d(-80, -52), Math.toRadians(-40), (pose2dDual, posePath, v) -> 80)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-48, -79), Math.toRadians(-40), (pose2dDual, posePath, v) -> 80)
                .strafeToLinearHeading(new Vector2d(-64, -65), Math.toRadians(-38), (pose2dDual, posePath, v) -> 80)
                .strafeToLinearHeading(new Vector2d(-66, -22), Math.toRadians(-37), (pose2dDual, posePath, v) -> 160)
                .stopAndAdd(Revolver.runSequence3ShotsNoShooter())
                .strafeToLinearHeading(new Vector2d(-65,-32), Math.toRadians(-38), (pose2dDual, posePath, v) -> 160)

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
