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
        // change to 63.5
        mBeginPose = new Pose2d(-15.5, 61, HeadingRad(-90));
    }

    @Override
    protected SequentialAction CreateAuto() {
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

        Pose2d parkPos = new Pose2d(-48, 60, HeadingRad(-90));

        Action takeS1ToChamber = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(chamberPos1, HeadingRad(-90))
                .build();

        Action backup = mDrive.actionBuilder(chamberPos1)
                .setTangent(HeadingRad(-90))
                .splineToLinearHeading(afterS1Score, HeadingRad(90))
                .build();

        Action driveToS2 = mDrive.actionBuilder(afterS1Score)
                .setTangent(HeadingRad(-135))
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

        Action pickupPauseS2 = mDrive.actionBuilder(finishSlideS3)
                .splineToLinearHeading(specPausePos, HeadingRad(0))
                .build();

        Action pickupS2 = mDrive.actionBuilder(specPausePos)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(specWallPos, HeadingRad(-90))
                .build();

        Action takeS2ToChamber = mDrive.actionBuilder(specWallPos)
                .setTangent(HeadingRad(-45))
                .splineToLinearHeading(chamberPos2, HeadingRad(-90))
                .build();

        Action pickupPauseS3 = mDrive.actionBuilder(chamberPos2)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(specPausePos, HeadingRad(180))
                .build();

        Action pickupS3 = mDrive.actionBuilder(specPausePos)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(specWallPos, HeadingRad(-90))
                .build();

        Action takeS3ToChamber = mDrive.actionBuilder(specWallPos)
                .setTangent(HeadingRad(-45))
                .splineToLinearHeading(chamberPos3, HeadingRad(-90))
                .build();

        Action pickupPauseS4 = mDrive.actionBuilder(chamberPos3)
                .splineToLinearHeading(specPausePos, HeadingRad(0))
                .build();

        Action takeS4ToChamber = mDrive.actionBuilder(specWallPos)
                .setTangent(HeadingRad(-45))
                .splineToLinearHeading(chamberPos4, HeadingRad(-90))
                .build();

        Action park = mDrive.actionBuilder(chamberPos4)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(parkPos, HeadingRad(180))
                .build();

        return new SequentialAction(
                // make sure claw is closed
                mArm.GetAction(ArmActions.RunAscent1),
                mClaw.GetAction(ClawActions.Close),
                ScoreActions(takeS1ToChamber),
                backup,
                new ParallelAction (
                        driveToS2,
                        mArm.GetAction(ArmActions.RunAutoSamplePush)
                ),
                pushS2ToObs,
                new ParallelAction(
                        mArm.GetBasketPostScore(15, 0),
                        driveToS3
                ),
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                pushS3ToObs,
                /*new ParallelAction (
                        mArm.GetBasketPostScore(15, 0),
                        driveToS4
                ),
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                pushS4ToObs,*/
                new ParallelAction (
                        mArm.GetAction(ArmActions.RunAscent1),
                        mSpecArm.GetAction(ArmActions.RunPickup),
                        pickupPauseS2
                ),
                new SleepAction(0.5),
                new ParallelAction(
                        mClaw.GetAction(ClawActions.Open),
                        pickupS2
                ),
                mClaw.GetAction(ClawActions.Close),
                new SleepAction(0.25),
                ScoreActions(takeS2ToChamber),
                PickupActions(pickupPauseS3, pickupS3),
                ScoreActions(takeS3ToChamber),
                park/*,
                PickupActions(pickupPauseS4, pickupSpec),
                ScoreActions(takeS4ToChamber),
                park,
                mArm.GetAction(ArmActions.RunHome)*/
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
                new ParallelAction(
                        mSpecArm.GetAction(ArmActions.RunPickup),
                        driveToSpecimen
                ),
                new SleepAction(0.5),
                driveToWall,
                mClaw.GetAction(ClawActions.Close),
                new SleepAction(0.25)
        );
    }
}
