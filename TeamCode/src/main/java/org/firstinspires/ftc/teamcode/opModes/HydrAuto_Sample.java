package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ClawActions;
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
        Pose2d basket = new Pose2d(54.5, 53.5, HeadingRad(-135));
        Pose2d s2 = new Pose2d(59, 51.25, HeadingRad(-90));
        Pose2d basketS2 = new Pose2d(54.5, 53.5, HeadingRad(-135));
        Pose2d s3 = new Pose2d(49.5, 51.5, HeadingRad(-88));
        Pose2d basketS3 = new Pose2d(54.5, 53.5, HeadingRad(-135));
        Pose2d s4 = new Pose2d(59, 48.5, HeadingRad(-62.5));
        Pose2d basketS4 = new Pose2d(54.5, 53.5, HeadingRad(-135));
        Pose2d park = new Pose2d(25, 10, HeadingRad(180));

        Action takeS1ToBasket = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(basket, HeadingRad(0))
                .build();

        Action driveToS2 = mDrive.actionBuilder(basket)
                .setTangent(HeadingRad(135))
                .splineToLinearHeading(s2, HeadingRad(135))
                .build();

        Action takeS2ToBasket = mDrive.actionBuilder(s2)
                .splineToLinearHeading(basketS2, HeadingRad(0))
                .build();

        Action driveToS3 = mDrive.actionBuilder(basketS2)
                .splineToLinearHeading(s3, HeadingRad(180))
                .build();

        Action takeS3ToBasket = mDrive.actionBuilder(s3)
                .splineToLinearHeading(basketS3, HeadingRad(0))
                .build();

        Action driveToS4 = mDrive.actionBuilder(basketS3)
                .setTangent(HeadingRad(-90))
                .splineToLinearHeading(s4, HeadingRad(-90))
                .build();

        Action takeS4ToBasket = mDrive.actionBuilder(s4)
                .splineToLinearHeading(basketS4, HeadingRad(-90))
                .build();

        Action goPark = mDrive.actionBuilder(basketS4)
                .setTangent(HeadingRad(270))
                .splineToLinearHeading(park, HeadingRad(180))
                .build();

        return new SequentialAction(
                new ParallelAction(
                    mArm.GetAction(ArmActions.RunPickup),
                    mSpecArm.GetAction(ArmActions.RunScoreLow),
                    mClaw.GetAction(ClawActions.Close)
                ),
                new ParallelAction(
                    ScoreActions(takeS1ToBasket),
                    mSpecArm.GetAction(ArmActions.RunPickup)
                ),
                PickupActions(driveToS2),
                ScoreActions(takeS2ToBasket),
                PickupActions(driveToS3),
                ScoreActions(takeS3ToBasket),
                PickupActions(driveToS4),
                ScoreActions(takeS4ToBasket),
                mArm.GetAction(ArmActions.RunHome)/*,
                new ParallelAction(
                        goPark,
                        mArm.GetAction(ArmActions.RunAscent1)
                )*/
        );
    }

    private SequentialAction ScoreActions(Action driveToBasket) {
        return new SequentialAction(
                new ParallelAction(
                        driveToBasket,
                        mArm.GetAction(ArmActions.RunScoreHigh)
                ),
                mIntake.GetAction(IntakeActions.OutContinuous),
                new SleepAction(0.4),
                new ParallelAction(
                        mArm.GetBasketPostScore(-10, 0.05),
                        mIntake.GetAction(IntakeActions.Stop)
                )
        );
    }

    private SequentialAction PickupActions(Action driveToSample) {
        return new SequentialAction(
                //new ParallelAction(
                mIntake.GetAction(IntakeActions.InStart),
                driveToSample,
                mArm.GetAction(ArmActions.RunAutoSamplePickup),
                //),
                mIntake.GetAction(IntakeActions.IntakeElement)
        );
    }
}
