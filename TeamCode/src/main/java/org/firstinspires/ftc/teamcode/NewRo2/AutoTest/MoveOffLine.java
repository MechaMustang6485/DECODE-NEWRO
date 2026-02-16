package org.firstinspires.ftc.teamcode.NewRo2.AutoTest;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous//working
public final class MoveOffLine extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TrajectoryActionBuilder move = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                ;


        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new ParallelAction(
                        move.build()
                )
        )
        ;
    }
}
