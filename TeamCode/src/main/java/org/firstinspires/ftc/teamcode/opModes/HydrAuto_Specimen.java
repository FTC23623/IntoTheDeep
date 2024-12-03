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

public class HydrAuto_Specimen extends HydrAuto {

    public HydrAuto_Specimen() {
        mElementType = ElementTypes.Specimen;
        mBeginPose = new Pose2d(-15.5, 63.5, HeadingRad(-90));
        mRunIntakeAtStart = false;
    }

    @Override
    protected SequentialAction CreateAuto() {
        Pose2d chamberPos1 = new Pose2d(-15.5, 32, HeadingRad(-90));
        Pose2d chamberPos2 = new Pose2d(-7.5, 31.5, HeadingRad(-90));
        Pose2d chamberPos3 = new Pose2d(-4, 31.5, HeadingRad(-90));
        Pose2d chamberPos4 = new Pose2d(-1, 31.5, HeadingRad(-90));

        Pose2d afterS1Score = new Pose2d(-15.5, 50, HeadingRad(-135));

        Pose2d specPausePos = new Pose2d(-44, 50, HeadingRad(-90));
        Pose2d specWallPos = new Pose2d(-44, 62, HeadingRad(-90));

        Pose2d parkPos = new Pose2d(-60, 60, HeadingRad(-90));

        Action takeS1ToChamber = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(chamberPos1, HeadingRad(-90))
                .build();

        Action backup = mDrive.actionBuilder(chamberPos1)
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(afterS1Score, HeadingRad(90))
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
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(specPausePos, HeadingRad(180))
                .build();

        Action takeS4ToChamber = mDrive.actionBuilder(specWallPos)
                .setTangent(HeadingRad(-45))
                .splineToLinearHeading(chamberPos4, HeadingRad(-90))
                .build();

        Action park = mDrive.actionBuilder(chamberPos3)
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
                // sweep or push the samples
                SamplesToObsZone(afterS1Score),
                // score specimens
                PickupActions(PickupS2(specPausePos), DriveToWall(specPausePos, specWallPos)),
                ScoreActions(takeS2ToChamber),
                PickupActions(pickupPauseS3, DriveToWall(specPausePos, specWallPos)),
                ScoreActions(takeS3ToChamber),
                //PickupActions(pickupPauseS4, DriveToWall(specPausePos, specWallPos)),
                //ScoreActions(takeS4ToChamber),
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
                        mArm.GetAction(ArmActions.RunAutoSpecSafe),
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
                driveToObsZone,
                mClaw.GetAction(ClawActions.Close),
                mSpecArm.GetAction(ArmActions.RunPickup),
                // open the claw
                // wait for the human player to align the specimen
                new ParallelAction(
                        mClaw.GetAction(ClawActions.Open),
                        new SleepAction(0.4)
                ),
                // drive back to the wall
                driveToWall,
                // close the claw
                mClaw.GetAction(ClawActions.Close),
                // wait for the claw to close
                new SleepAction(0.25)
        );
    }

    protected SequentialAction SamplesToObsZone(Pose2d startPos) {
        return null;
    }

    protected Action PickupS2(Pose2d endPos) {
        return null;
    }
}
