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
            Pose2d basket = new Pose2d(54.5, 53.5, HeadingRad(-135));
            Pose2d s2 = new Pose2d(59, 51.25, HeadingRad(-90));
            Pose2d basketS2 = new Pose2d(54.5, 53.5, HeadingRad(-135));
            Pose2d s3 = new Pose2d(49.5, 51.5, HeadingRad(-88));
            Pose2d basketS3 = new Pose2d(54.5, 53.5, HeadingRad(-135));
            Pose2d s4 = new Pose2d(59, 48.5, HeadingRad(-62.5));
            Pose2d basketS4 = new Pose2d(54.5, 53.5, HeadingRad(-135));
            Pose2d park = new Pose2d(25, 10, HeadingRad(180));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(39.5, 63.5, HeadingRad(-90)))
                    .splineToLinearHeading(basket, HeadingRad(0))
                            .setTangent(HeadingRad(-90))
                    .splineToLinearHeading(s2,HeadingRad(-90))
                            .setTangent(HeadingRad(90))
                    .splineToLinearHeading(basketS2,HeadingRad(90))
                            .setTangent(HeadingRad(-90))
                    .splineToLinearHeading(s3,HeadingRad(-90))
                            .setTangent(HeadingRad(90))
                    .splineToLinearHeading(basketS3,HeadingRad(90))
                    .setTangent(HeadingRad(-90))
                    .splineToLinearHeading(s4,HeadingRad(-90))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(basketS4,HeadingRad(90))
                    .build());
        } else if (false) {
            Pose2d chamberPos1 = new Pose2d(-15.5, 32, HeadingRad(-90));
            Pose2d chamberPos2 = new Pose2d(-13.5, 31.5, HeadingRad(-90));
            Pose2d chamberPos3 = new Pose2d(-10, 31.5, HeadingRad(-90));
            Pose2d chamberPos4 = new Pose2d(-9.5, 31.5, HeadingRad(-90));

            Pose2d afterS1Score = new Pose2d(-15.5, 50, HeadingRad(-135));

            Pose2d startSlideS2 = new Pose2d(-30, 40, HeadingRad(-135));
            Pose2d finishSlideS2 = new Pose2d(-30, 45, HeadingRad(-230));

            Pose2d startSlideS3 = new Pose2d(-40, 40, HeadingRad(-135));
            Pose2d finishSlideS3 = new Pose2d(-40, 45, HeadingRad(-240));

            Pose2d startSlideS4 = new Pose2d(-48, 40, HeadingRad(-135));
            Pose2d finishSlideS4 = new Pose2d(-50, 45, HeadingRad(-250));

            Pose2d specPausePos = new Pose2d(-44, 50, HeadingRad(-90));
            Pose2d specWallPos = new Pose2d(-44, 62, HeadingRad(-90));

            Pose2d parkPos = new Pose2d(-60, 60, HeadingRad(-90));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15.5, 63.5, HeadingRad(-90)))
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
                    .splineToLinearHeading(specWallPos, HeadingRad(-90))
                    .setTangent(HeadingRad(-45))
                    .splineToLinearHeading(chamberPos2, HeadingRad(-90))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(specPausePos, HeadingRad(180))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(specWallPos, HeadingRad(-90))
                    .setTangent(HeadingRad(-45))
                    .splineToLinearHeading(chamberPos3, HeadingRad(-90))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(parkPos, HeadingRad(180))
                    .build());
        } else if (true) {
            Pose2d chamberPos1 = new Pose2d(-15.5, 32, HeadingRad(-90));
            Pose2d chamberPos2 = new Pose2d(-7.5, 31.5, HeadingRad(-90));
            Pose2d chamberPos3 = new Pose2d(-4, 31.5, HeadingRad(-90));
            Pose2d chamberPos4 = new Pose2d(-1, 31.5, HeadingRad(-90));

            Pose2d afterS1Score = new Pose2d(-15.5, 50, HeadingRad(-135));

            Pose2d startPushS2 = new Pose2d(-50, 18, HeadingRad(-90));
            Pose2d finishPushS2 = new Pose2d(-50, 54, HeadingRad(-90));

            Pose2d startPushS3 = new Pose2d(-60, 18, HeadingRad(-90));
            Pose2d finishPushS3 = new Pose2d(-60, 54, HeadingRad(-90));

            Pose2d startPushS4 = new Pose2d(-66, 12, HeadingRad(-90));
            Pose2d finishPushS4 = new Pose2d(-66, 60, HeadingRad(-90));

            Pose2d specPausePos = new Pose2d(-48, 50, HeadingRad(-90));
            Pose2d specWallPos = new Pose2d(-48, 63, HeadingRad(-90));

            Pose2d parkPos = new Pose2d(-60, 60, HeadingRad(-90));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15.5, 63.5, HeadingRad(-90)))
                    .splineToLinearHeading(chamberPos1, HeadingRad(-90))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(afterS1Score, HeadingRad(90))
                    .setTangent(HeadingRad(180))
                    .splineToLinearHeading(startPushS2, HeadingRad(180))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(finishPushS2, HeadingRad(90))
                    .setTangent(HeadingRad(-90))
                    .splineToLinearHeading(startPushS3, HeadingRad(180))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(finishPushS3, HeadingRad(90))
                    //.splineToLinearHeading(startSlideS4, HeadingRad(180))
                    //.splineToLinearHeading(finishSlideS4, HeadingRad(90))
                    .setTangent(HeadingRad(0))
                    .splineToLinearHeading(specPausePos, HeadingRad(0))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(specWallPos,HeadingRad(90))
                    .setTangent(HeadingRad(-45))
                    .splineToLinearHeading(chamberPos2, HeadingRad(-90))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(specPausePos, HeadingRad(180))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(specWallPos, HeadingRad(90))
                    .setTangent(HeadingRad(-45))
                    .splineToLinearHeading(chamberPos3, HeadingRad(-90))
                    //.setTangent(HeadingRad(90))
                    //.splineToLinearHeading(specPausePos, HeadingRad(180))
                    //.setTangent(HeadingRad(90))
                    //.splineToLinearHeading(specWallPos, HeadingRad(-90))
                    //.setTangent(HeadingRad(-45))
                    //.splineToLinearHeading(chamberPos4, HeadingRad(-90))
                    .setTangent(HeadingRad(90))
                    .splineToLinearHeading(parkPos, HeadingRad(180))
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
