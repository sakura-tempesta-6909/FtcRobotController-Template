package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 60.0, Math.toRadians(90)))
                        .waitSeconds(1.0)
                        .lineToLinearHeading(new Pose2d(0.0, 35.0, Math.toRadians(90)))
                        .addTemporalMarker(() ->{
                            // スライダーを伸ばし、フックに標本を引っかける
                        })
                        .waitSeconds(2.0)
                        .lineToLinearHeading(new Pose2d(-35.0, 35.0, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-35.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-45.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-45.0, 60.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-45.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-55.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-55.0, 60.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-55.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-60.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-60.0, 60.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-40.0, 60.0, Math.toRadians(-90)))
                        .waitSeconds(1.0)
                        .lineToLinearHeading(new Pose2d(-40.0, 65.0, Math.toRadians(-90)))
                        .addTemporalMarker(() ->{
                            // 標本をつかむ
                        })
                        .waitSeconds(2.0)
                        .lineToLinearHeading(new Pose2d(-40.0, 60.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(0.0, 35.0, Math.toRadians(90)))
                        .addTemporalMarker(() ->{
                            // スライダーを伸ばし、フックに標本を引っかける
                        })
                        .waitSeconds(2.0)
                        .lineToLinearHeading(new Pose2d(-40.0, 60.0, Math.toRadians(90)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}