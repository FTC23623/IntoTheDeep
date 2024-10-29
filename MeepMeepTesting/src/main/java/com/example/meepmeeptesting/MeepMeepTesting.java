package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, 63, HeadingRad(-90)))
                .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                .splineToLinearHeading(new Pose2d(59,46,HeadingRad(-90)),HeadingRad(-90))
                .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                .splineToLinearHeading(new Pose2d(49,46, HeadingRad(-90)),HeadingRad(-90))
                .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                .splineToLinearHeading(new Pose2d(51,26,HeadingRad(0)),HeadingRad(90))
                .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                       // .lineTo(new Pose2d(56,15, HeadingRad(180)), HeadingRad(180))
               // .splineToLinearHeading(new Pose2d(38,10,HeadingRad(180)),HeadingRad(-90))
                .splineToLinearHeading(new Pose2d(46,54, HeadingRad(-90)), HeadingRad(0))
                .splineToLinearHeading(new Pose2d(25,10,HeadingRad(180)),HeadingRad(180))
                .build());

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
