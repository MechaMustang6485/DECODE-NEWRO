package org.firstinspires.ftc.teamcode.NEWRO.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;




@Config
@Autonomous
public final class backB extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        //3 artifact
                        .lineToX(10)
                        .strafeToLinearHeading(new Vector2d(64, 26.8), Math.toRadians(89), (pose2dDual, posePath, v) -> 50)
                        .strafeToLinearHeading(new Vector2d(64, 55), Math.toRadians(89), (pose2dDual, posePath, v) -> 50)
                        .strafeToLinearHeading(new Vector2d(27, 1), Math.toRadians(89), (pose2dDual, posePath, v) -> 50)
                        .strafeToLinearHeading(new Vector2d(103.4, 50.6), Math.toRadians(89), (pose2dDual, posePath, v) -> 50)
                        .strafeToLinearHeading(new Vector2d(103.4, 65.2), Math.toRadians(89), (pose2dDual, posePath, v) -> 50)
                        .strafeToLinearHeading(new Vector2d(27,12), Math.toRadians(89), (pose2dDual, posePath, v) -> 50)
                        .lineToX(65)
                        /*
                        .strafeToLinearHeading(new Vector2d(33.2, -85.1), Math.toRadians(-89), (pose2dDual, posePath, v) -> 80)
                        .strafeToLinearHeading(new Vector2d(64.1, -86.5), Math.toRadians(-89), (pose2dDual, posePath, v) -> 80)
                        .strafeToLinearHeading(new Vector2d(1, 1), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                        .strafeToLinearHeading(new Vector2d(35.9, -132.4), Math.toRadians(-89), (pose2dDual, posePath, v) -> 80)
                        .strafeToLinearHeading(new Vector2d(62.5, -132.9), Math.toRadians(-89), (pose2dDual, posePath, v) -> 80)
                        .strafeToLinearHeading(new Vector2d(15.5, -23.3), Math.toRadians(0), (pose2dDual, posePath, v) -> 80)
                         */
                        .build());


    }




}






