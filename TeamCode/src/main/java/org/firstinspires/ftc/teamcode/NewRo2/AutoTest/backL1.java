package org.firstinspires.ftc.teamcode.NewRo2.AutoTest;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Config
@Autonomous
public final class backL1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        //3 artifact
//                        .lineToX(10)
//                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(63.9, 49.7), Math.toRadians(89), (pose2dDual, posePath, v) -> 40)
                        .strafeToLinearHeading(new Vector2d(63.0, 77.4), Math.toRadians(89), (pose2dDual, posePath, v) -> 40)
                        .strafeToLinearHeading(new Vector2d(1, 1), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(104.9, 66.7), Math.toRadians(89), (pose2dDual, posePath, v) -> 40)
                        .strafeToLinearHeading(new Vector2d(104.4, 82.2), Math.toRadians(89), (pose2dDual, posePath, v) -> 40)
                        .strafeToLinearHeading(new Vector2d(1, 1), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
//                        .strafeToLinearHeading(new Vector2d(35.9, -132.4), Math.toRadians(89), (pose2dDual, posePath, v) -> 80)
//                        .strafeToLinearHeading(new Vector2d(62.5, -132.9), Math.toRadians(89), (pose2dDual, posePath, v) -> 80)
//                        .strafeToLinearHeading(new Vector2d(15.5, -23.3), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                        .build());


    }
    public class revolverAction implements Action {
        DcMotorEx revolver;
        int RevolverPosition;

        public revolverAction(DcMotorEx R, int p){
            this.revolver = R;
            this.RevolverPosition = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            revolver.setTargetPosition(RevolverPosition);
            return false;
        }
    }



}






