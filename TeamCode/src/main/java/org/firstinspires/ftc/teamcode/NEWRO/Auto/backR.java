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
public final class backR extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        //3 artifact
                        .lineToX(10)//shoot1
                        .strafeToLinearHeading(new Vector2d(32.7, -39.2), Math.toRadians(-89), (pose2dDual, posePath, v) -> 50)
                        //intake
                        .strafeToLinearHeading(new Vector2d(32.7, -58), Math.toRadians(-89), (pose2dDual, posePath, v) -> 50)
                        .strafeToLinearHeading(new Vector2d(-4, 1), Math.toRadians(-89), (pose2dDual, posePath, v) -> 50)//shoot
                        //intake
                        .strafeToLinearHeading(new Vector2d(67.9, -39.7), Math.toRadians(-89), (pose2dDual, posePath, v) -> 50)

                        .strafeToLinearHeading(new Vector2d(67.9, -56), Math.toRadians(-89), (pose2dDual, posePath, v) -> 50)//shoot
                        .strafeToLinearHeading(new Vector2d(-8,1), Math.toRadians(-89), (pose2dDual, posePath, v) -> 50)
                        .lineToX(10)
                        .build());


    }




}






