package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

@Autonomous(name = "HydrAuto_Sample", preselectTeleOp = "HyDrive_Sample")
public class HydrAuto_Sample extends HydrAuto {

    public HydrAuto_Sample() {
        mElementType = ElementTypes.Sample;
        mBeginPose = new Pose2d(39.5, 63.5, HeadingRad(-90));
        mRunIntakeAtStart = true;
    }

    @Override
    protected SequentialAction CreateAuto() {
        Pose2d basket = new Pose2d(53, 52, HeadingRad(-135));
        Pose2d s2 = new Pose2d(59, 51, HeadingRad(-90));
        Pose2d s3 = new Pose2d(49, 51.5, HeadingRad(-87));
        Pose2d s4 = new Pose2d(52, 45, HeadingRad(-45));
        Pose2d park = new Pose2d(25, 10, HeadingRad(180));

        Action takeS1ToBasket = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(basket, HeadingRad(0))
                .build();

        Action driveToS2 = mDrive.actionBuilder(basket)
                .splineToLinearHeading(s2, HeadingRad(-90))
                .build();

        Action takeS2ToBasket = mDrive.actionBuilder(s2)
                .splineToLinearHeading(basket, HeadingRad(0))
                .build();

        Action driveToS3 = mDrive.actionBuilder(basket)
                .splineToLinearHeading(s3, HeadingRad(-90))
                .build();

        Action takeS3ToBasket = mDrive.actionBuilder(s3)
                .splineToLinearHeading(basket, HeadingRad(0))
                .build();

        Action driveToS4 = mDrive.actionBuilder(basket)
                .splineToLinearHeading(s4, HeadingRad(90))
                .build();

        Action takeS4ToBasket = mDrive.actionBuilder(s4)
                .splineToLinearHeading(basket, HeadingRad(0))
                .build();

        Action goPark = mDrive.actionBuilder(basket)
                .setTangent(HeadingRad(270))
                .splineToLinearHeading(park, HeadingRad(180))
                .build();

        return new SequentialAction(
                mArm.GetAction(ArmActions.RunPickup),
                ScoreActions(takeS1ToBasket),
                PickupActions(driveToS2),
                ScoreActions(takeS2ToBasket),
                PickupActions(driveToS3),
                ScoreActions(takeS3ToBasket),
                PickupActions(driveToS4),
                ScoreActions(takeS4ToBasket),
                new ParallelAction(
                        goPark,
                        mArm.GetAction(ArmActions.RunAscent1)
                )
        );
    }

    private SequentialAction ScoreActions(Action driveToBasket) {
        return new SequentialAction(
                new ParallelAction(
                        driveToBasket,
                        mArm.GetAction(ArmActions.RunScoreHigh)
                ),
                mIntake.GetAction(IntakeActions.OutContinuous),
                new SleepAction(0.5),
                new ParallelAction(
                        mArm.GetBasketPostScore(-10, 0.05),
                        mIntake.GetAction(IntakeActions.Stop)
                )
        );
    }

    private SequentialAction PickupActions(Action driveToSample) {
        return new SequentialAction(
                new ParallelAction(
                        driveToSample,
                        mIntake.GetAction(IntakeActions.InStart),
                        mArm.GetAction(ArmActions.RunAutoSamplePickup)
                ),
                mIntake.GetAction(IntakeActions.IntakeElement)
        );
    }
}
