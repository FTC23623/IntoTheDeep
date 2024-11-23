package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ClawActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

@Autonomous(name="HydrAuto_Obs", preselectTeleOp = "HyDrive_Specimen")
public class HydrAuto_Obs extends HydrAuto {

    public HydrAuto_Obs() {
        mElementType = ElementTypes.Specimen;
        mBeginPose = new Pose2d(-15, 63.5, HeadingRad(-90));
    }

    @Override
    protected SequentialAction CreateAuto() {
        Pose2d chamberPos1 = new Pose2d(-15, 40, HeadingRad(-90));
        Pose2d chamberPos2 = new Pose2d(-13, 40, HeadingRad(-90));
        Pose2d chamberPos3 = new Pose2d(-11, 40, HeadingRad(-90));
        Pose2d chamberPos4 = new Pose2d(-9, 40, HeadingRad(-90));

        Pose2d startSlideS2 = new Pose2d(-30, 40, HeadingRad(-135));
        Pose2d finishSlideS2 = new Pose2d(-30, 42, HeadingRad(-225));

        Pose2d startSlideS3 = new Pose2d(-40, 40, HeadingRad(-135));
        Pose2d finishSlideS3 = new Pose2d(-40, 42, HeadingRad(-225));

        Pose2d startSlideS4 = new Pose2d(-50, 40, HeadingRad(-135));
        Pose2d finishSlideS4 = new Pose2d(-50, 42, HeadingRad(-225));

        Pose2d specPausePos = new Pose2d(-41, 50, HeadingRad(-90));
        Pose2d specWallPos = new Pose2d(-41, 62, HeadingRad(-90));

        Pose2d parkPos = new Pose2d(-48, 60, HeadingRad(-90));

        Action takeS1ToChamber = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(chamberPos1, HeadingRad(-90))
                .build();

        Action driveToS2 = mDrive.actionBuilder(chamberPos1)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(startSlideS2, HeadingRad(-135))
                .build();

        Action pushS2ToObs = mDrive.actionBuilder(startSlideS2)
                .splineToLinearHeading(finishSlideS2, HeadingRad(-90))
                .build();

        Action driveToS3 = mDrive.actionBuilder(finishSlideS2)
                .splineToLinearHeading(startSlideS3, HeadingRad(180))
                .build();

        Action pushS3ToObs = mDrive.actionBuilder(startSlideS3)
                .splineToLinearHeading(finishSlideS3, HeadingRad(-90))
                .build();

        Action driveToS4 = mDrive.actionBuilder(finishSlideS3)
                .splineToLinearHeading(startSlideS4, HeadingRad(180))
                .build();

        Action pushS4ToObs = mDrive.actionBuilder(startSlideS4)
                .splineToLinearHeading(finishSlideS4, HeadingRad(-90))
                .build();

        Action pickupPauseS2 = mDrive.actionBuilder(finishSlideS4)
                .splineToLinearHeading(specPausePos, HeadingRad(0))
                .build();

        Action pickupSpec = mDrive.actionBuilder(specPausePos)
                .splineToLinearHeading(specWallPos, HeadingRad(-90))
                .build();

        Action takeS2ToChamber = mDrive.actionBuilder(specWallPos)
                .splineToLinearHeading(chamberPos2, HeadingRad(0))
                .build();

        Action pickupPauseS3 = mDrive.actionBuilder(chamberPos2)
                .splineToLinearHeading(specPausePos, HeadingRad(0))
                .build();

        Action takeS3ToChamber = mDrive.actionBuilder(specWallPos)
                .splineToLinearHeading(chamberPos3, HeadingRad(0))
                .build();

        Action pickupPauseS4 = mDrive.actionBuilder(chamberPos3)
                .splineToLinearHeading(specPausePos, HeadingRad(0))
                .build();

        Action takeS4ToChamber = mDrive.actionBuilder(specWallPos)
                .splineToLinearHeading(chamberPos4, HeadingRad(0))
                .build();

        Action park = mDrive.actionBuilder(chamberPos4)
                .splineToLinearHeading(parkPos, HeadingRad(180))
                .build();

        return new SequentialAction(
                // make sure claw is closed
                mClaw.GetAction(ClawActions.Close),
                ScoreActions(takeS1ToChamber),
                new ParallelAction (
                        driveToS2,
                        mArm.GetAction(ArmActions.RunAutoSamplePush)
                ),
                pushS2ToObs,
                new ParallelAction(
                        mArm.GetBasketPostScore(5, 0),
                        driveToS3
                ),
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                pushS3ToObs,
                new ParallelAction (
                        mArm.GetBasketPostScore(5, 0),
                        driveToS4
                ),
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                pushS4ToObs,
                pickupPauseS2,
                new SleepAction(0.5),
                new ParallelAction(
                        mClaw.GetAction(ClawActions.Open),
                        pickupSpec
                ),
                ScoreActions(takeS2ToChamber),
                PickupActions(pickupPauseS3, pickupSpec),
                ScoreActions(takeS3ToChamber),
                PickupActions(pickupPauseS4, pickupSpec),
                ScoreActions(takeS4ToChamber),
                park,
                mArm.GetAction(ArmActions.RunHome)
        );
    }

    private SequentialAction ScoreActions(Action driveToChamber) {
        return new SequentialAction(
                new ParallelAction(
                        mSpecArm.GetAction(ArmActions.RunScoreHigh),
                        driveToChamber
                ),
                mSpecArm.GetAction(ArmActions.RunScoreHighScore),
                mClaw.GetAction(ClawActions.Open)
        );
    }

    private SequentialAction PickupActions(Action driveToSpecimen, Action driveToWall) {
        return new SequentialAction(
                mClaw.GetAction(ClawActions.Close),
                new ParallelAction(
                        mSpecArm.GetAction(ArmActions.RunPickup),
                        driveToSpecimen
                ),
                new SleepAction(0.5),
                new ParallelAction(
                        mClaw.GetAction(ClawActions.Open),
                        driveToWall
                ),
                mClaw.GetAction(ClawActions.Close),
                mSpecArm.GetAction(ArmActions.RunScoreHigh)
        );
    }
}
