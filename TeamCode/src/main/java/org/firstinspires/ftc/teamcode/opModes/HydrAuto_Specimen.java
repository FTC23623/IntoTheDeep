package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ClawActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Autonomous(name="HydrAuto_Specimen", preselectTeleOp = "HyDrive_Specimen")
public class HydrAuto_Specimen extends HydrAuto {

    public HydrAuto_Specimen() {
        mElementType = ElementTypes.Specimen;
        // change to 63.5
        mBeginPose = new Pose2d(-15.5, 61, HeadingRad(-90));
        mRunIntakeAtStart = false;
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

        Action takeS2ToChamber = mDrive.actionBuilder(specWallPos)
                .setTangent(HeadingRad(-45))
                .splineToLinearHeading(chamberPos2, HeadingRad(-90))
                .build();

        Action pickupPauseS3 = mDrive.actionBuilder(chamberPos2)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(specPausePos, HeadingRad(180))
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
                // make sure claw is closed on the preloaded specimen
                mClaw.GetAction(ClawActions.Close),
                // score the first specimen
                ScoreActions(takeS1ToChamber),
                // back away so we don't hit the submersible
                // lower the specimen arm
                new ParallelAction(
                        backup,
                        mSpecArm.GetAction(ArmActions.RunPickup)
                ),
                // drive to the sample on the floor
                // bring the arm down to push
                new ParallelAction (
                        driveToS2,
                        mArm.GetAction(ArmActions.RunAutoSamplePush)
                ),
                // push the sample to the observation zone
                pushS2ToObs,
                // lift the arm to go over the sample
                // drive to the next sample
                new ParallelAction(
                        mArm.GetBasketPostScore(15, 0),
                        driveToS3
                ),
                // lower the arm to push
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                // push the sample to the observation zone
                pushS3ToObs,
                /*new ParallelAction (
                        mArm.GetBasketPostScore(15, 0),
                        driveToS4
                ),
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                pushS4ToObs,*/
                // score specimens
                PickupActions(pickupPauseS2, DriveToWall(specPausePos, specWallPos)),
                ScoreActions(takeS2ToChamber),
                PickupActions(pickupPauseS3, DriveToWall(specPausePos, specWallPos)),
                ScoreActions(takeS3ToChamber),/*,
                PickupActions(pickupPauseS4, DriveToWall(specPausePos, specWallPos)),
                ScoreActions(takeS4ToChamber),*/
                park,
                mArm.GetAction(ArmActions.RunHome)
        );
    }

    private Action DriveToWall(Pose2d specPausePos, Pose2d specWallPos) {
        return mDrive.actionBuilder(specPausePos)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(specWallPos, HeadingRad(-90))
                .build();
    }

    private SequentialAction ScoreActions(Action driveToChamber) {
        return new SequentialAction(
                // raise specimen arm to scoring position
                // make sure the main arm is up so it does not hit the submersible
                // drive to scoring position
                new ParallelAction(
                        mSpecArm.GetAction(ArmActions.RunScoreHigh),
                        mArm.GetAction(ArmActions.RunAscent1),
                        driveToChamber
                ),
                // score the specimen
                mSpecArm.GetAction(ArmActions.RunScoreHighScore),
                // open the claw
                mClaw.GetAction(ClawActions.Open)
        );
    }

    private SequentialAction PickupActions(Action driveToObsZone, Action driveToWall) {
        return new SequentialAction(
                // get specimen arm back to pickup position
                // drive to the observation zone
                new ParallelAction(
                        mSpecArm.GetAction(ArmActions.RunPickup),
                        driveToObsZone
                ),
                // open the claw
                // wait for the human player to align the specimen
                new ParallelAction(
                        mClaw.GetAction(ClawActions.Open),
                        new SleepAction(0.5)
                ),
                // drive back to the wall
                driveToWall,
                // close the claw
                mClaw.GetAction(ClawActions.Close),
                // wait for the claw to close
                new SleepAction(0.25)
        );
    }
}
