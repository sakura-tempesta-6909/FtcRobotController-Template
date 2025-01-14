package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 60.0, Math.toRadians(90)))
                        .waitSeconds(1.0)
                        .addTemporalMarker(() -> {
                        })
                        // 前に進む
                        .lineToLinearHeading(new Pose2d(0.0, 30.0, Math.toRadians(90)))
                        .addTemporalMarker(() -> {
                            // スライダーを伸ばし、フックに標本を引っかける
                        })
                        .waitSeconds(0.5)
                        // ちょっと下がる
                        .setAccelConstraint(new ProfileAccelerationConstraint(20))
                        .lineToLinearHeading(new Pose2d(0.0, 40.0, Math.toRadians(90)))
                        .addTemporalMarker(() -> {
                        })
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(0.0, 48.0, Math.toRadians(90)))
                        .addTemporalMarker(() -> {
                        })
                        .waitSeconds(0.2)
                        .resetAccelConstraint()
                        .addTemporalMarker(() -> {
                        })
//                // 横に移動する
                        .lineToLinearHeading(new Pose2d(-35.0, 48.0, Math.toRadians(90)))
//                // 前に移動する
                        .lineToLinearHeading(new Pose2d(-35.0, 10.0, Math.toRadians(-90)))
                        // 横に行く
                        .lineToLinearHeading(new Pose2d(-50.0, 10.0, Math.toRadians(-90)))
//                        // 後ろに移動する
                        .lineToLinearHeading(new Pose2d(-50.0, 60.0, Math.toRadians(-90)))
                        // 横に移動しながら、前に進む
//                        .lineToConstantHeading(new Vector2d(-55.0, 60.0))
                        // 元の位置に戻る
                        .lineToLinearHeading(new Pose2d(-50.0, 10.0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-60.0, 10.0, Math.toRadians(-90)))
                        // 横に移動しながら、後ろに進む
                        .lineToConstantHeading(new Vector2d(-60.0, 60.0))
                        // 横に移動する
//                        .lineToLinearHeading(new Pose2d(-60.0, 60.0, Math.toRadians(-90)))
                        // 前に移動する
                        .lineToLinearHeading(new Pose2d(-40.0, 55.0, Math.toRadians(-90)))
                        // 標本をつかむ準備をする
                        .waitSeconds(1.0)
                        // 標本に近づく
                        .lineToLinearHeading(new Pose2d(-40.0, 60.0, Math.toRadians(-90)))
//                // 標本をつかむ
//                .addTemporalMarker(() -> {
//                    // 標本をつかむ
//                })
//                .waitSeconds(2.0)
////                        // 後ろに下がる
////                        .lineToLinearHeading(new Pose2d(-40.0, 60.0, Math.toRadians(-90)))
////                        // 引っかける一に移動する
////                        .lineToLinearHeading(new Pose2d(0.0, 35.0, Math.toRadians(90)))
//                .lineToSplineHeading(new Pose2d(10.0, 35.0, Math.toRadians(90)))
//                .addTemporalMarker(() -> {
//                    // スライダーを伸ばし、フックに標本を引っかける
//                })
//                .waitSeconds(2.0)
//                .lineToLinearHeading(new Pose2d(-40.0, 60.0, Math.toRadians(90)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}