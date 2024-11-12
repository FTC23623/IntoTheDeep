package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

@Autonomous(name = "HydrAuto_Net", preselectTeleOp = "HyDrive_Sample")
public class HydrAuto_Net extends HydrAuto {

    public HydrAuto_Net() {
        mElementType = ElementTypes.Sample;
        mBeginPose = new Pose2d(39.5, 63.5, HeadingRad(-90));
    }

    @Override
    protected SequentialAction CreateAuto() {
        Pose2d basket = new Pose2d(56, 50, HeadingRad(-135));
        Pose2d s2 = new Pose2d(64, 50, HeadingRad(-90));
        Pose2d s3 = new Pose2d(49, 50, HeadingRad(-90));
        Pose2d s4 = new Pose2d(52, 26, HeadingRad(0));

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

        Action park = mDrive.actionBuilder(basket)
                .splineToLinearHeading(new Pose2d(58, 56, HeadingRad(-90)), HeadingRad(0))
                .splineToLinearHeading(new Pose2d(25, 10, HeadingRad(180)), HeadingRad(180))
                .build();

        return new SequentialAction(
                mArm.GetAction(ArmActions.RunPickup),
                takeS1ToBasket,
                ScoreActions(),
                driveToS2,
                PickupActions(),
                takeS2ToBasket,
                ScoreActions(),
                driveToS3,
                PickupActions(),
                takeS3ToBasket,
                ScoreActions(),
                mArm.GetAction(ArmActions.RunHome)
        );
    }

    private SequentialAction ScoreActions() {
        return new SequentialAction(
                mArm.GetAction(ArmActions.RunScoreHigh),
                mIntake.GetAction(IntakeActions.OutContinuous),
                new SleepAction(1.0),
                mArm.GetBasketPostScore(-10, 0.05),
                mIntake.GetAction(IntakeActions.Stop),
                mArm.GetAction(ArmActions.RunCarry),
                mArm.GetAction(ArmActions.RunPickup)
        );
    }

    private SequentialAction PickupActions() {
        return new SequentialAction(
            mIntake.GetAction(IntakeActions.InStart),
            mArm.GetAction(ArmActions.RunAutoSamplePickup),
            mIntake.GetAction(IntakeActions.IntakeElement),
            mArm.GetAction(ArmActions.RunCarry)
        );
    }
}
