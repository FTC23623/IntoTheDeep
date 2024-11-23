package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), HeadingRad(180), 15)
                .build();

        if (false) {
            Pose2d basket = new Pose2d(56, 50, HeadingRad(-135));
            Pose2d s2 = new Pose2d(52, 47, HeadingRad(-105));
            Pose2d s3 = new Pose2d(52, 47, HeadingRad(-85));
            Pose2d s4 = new Pose2d(52, 26, HeadingRad(0));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(39.5, 63.5, HeadingRad(-90)))
                    .splineToLinearHeading(basket, HeadingRad(0))
                    .splineToLinearHeading(s2,HeadingRad(-90))
                    /*.splineToLinearHeading(basket,HeadingRad(0))
                    .splineToLinearHeading(s3,HeadingRad(-90))
                    .splineToLinearHeading(basket,HeadingRad(0))/*
                    /*.splineToLinearHeading(s4,HeadingRad(90))
                    .splineToLinearHeading(basket,HeadingRad(0))*/
                    /*.splineToLinearHeading(new Pose2d(48, 38, HeadingRad(180)), HeadingRad(-90))
                    .splineToLinearHeading(new Pose2d(25,10,HeadingRad(180)),HeadingRad(180))*/
                    .build());
        } else if (true) {
            Pose2d chamberPos1 = new Pose2d(-15.5, 31.5, HeadingRad(-90));
            Pose2d chamberPos2 = new Pose2d(-13.5, 31.5, HeadingRad(-90));
            Pose2d chamberPos3 = new Pose2d(-11.5, 31.5, HeadingRad(-90));
            Pose2d chamberPos4 = new Pose2d(-9.5, 31.5, HeadingRad(-90));

            Pose2d afterS1Score = new Pose2d(-15.5, 50, HeadingRad(-135));

            Pose2d startSlideS2 = new Pose2d(-30, 40, HeadingRad(-135));
            Pose2d finishSlideS2 = new Pose2d(-30, 42, HeadingRad(-225));

            Pose2d startSlideS3 = new Pose2d(-40, 40, HeadingRad(-135));
            Pose2d finishSlideS3 = new Pose2d(-40, 42, HeadingRad(-225));

            Pose2d startSlideS4 = new Pose2d(-50, 40, HeadingRad(-135));
            Pose2d finishSlideS4 = new Pose2d(-50, 42, HeadingRad(-225));

            Pose2d specPausePos = new Pose2d(-41, 50, HeadingRad(-90));
            Pose2d specWallPos = new Pose2d(-41, 62, HeadingRad(-90));

            Pose2d parkPos = new Pose2d(-48, 60, HeadingRad(-90));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15.5, 61, HeadingRad(-90)))
                    .splineToLinearHeading(chamberPos1, HeadingRad(-90))
                    .setTangent(HeadingRad(-90))
                    .splineToLinearHeading(afterS1Score, HeadingRad(90))
                    .setTangent(HeadingRad(-135))
                    .splineToLinearHeading(startSlideS2, HeadingRad(-135))
                    .splineToLinearHeading(finishSlideS2, HeadingRad(-90))
                    .splineToLinearHeading(startSlideS3, HeadingRad(180))
                    .splineToLinearHeading(finishSlideS3, HeadingRad(-90))
                    //.splineToLinearHeading(startSlideS4, HeadingRad(180))
                    //.splineToLinearHeading(finishSlideS4, HeadingRad(-90))
                    .splineToLinearHeading(specPausePos, HeadingRad(0))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(specWallPos,HeadingRad(-90))
                            .setTangent(HeadingRad(-45))
                    .splineToLinearHeading(chamberPos2, HeadingRad(-90))
                            .setTangent(HeadingRad(90))
                            .splineToLinearHeading(specPausePos, HeadingRad(180))
                    .build());
        } else {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -38, HeadingRad(90)))
                    .setTangent(HeadingRad(-90))
                    .splineToLinearHeading(new Pose2d(-30, -38, HeadingRad(90)), HeadingRad(90))
                    .build());
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }
}
