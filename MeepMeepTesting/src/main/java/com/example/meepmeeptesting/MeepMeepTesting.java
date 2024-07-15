package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        60,
                        60,
                        Math.toRadians(270),
                        Math.toRadians(180),
                        16
                )
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();

        DriveShim drive = robot.getDrive();
        Action startTrajectory = drive.actionBuilder(new Pose2d(11.8, 61.7, Math.toRadians(-90)))
                .lineToY(37)
                .build();

        Action boardTrajectory = drive.actionBuilder(new Pose2d(11.8, 37, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .turnTo(47.1)
                .strafeToConstantHeading(new Vector2d(50, 31))
                .build();
        Action stackTrajectory = drive.actionBuilder(new Pose2d(49, 36, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-57,40))
                .build();
        robot.runAction(new SequentialAction(
                startTrajectory,
                new SleepAction(1.5),
                boardTrajectory,
                new SleepAction(2.5),
                stackTrajectory,
                new SleepAction(3.5)
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}