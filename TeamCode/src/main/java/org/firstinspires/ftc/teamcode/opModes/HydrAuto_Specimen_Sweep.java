package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.ArmActions;

@Autonomous(name="HydrAuto_Specimen_Sweep", preselectTeleOp = "HyDrive_Specimen")
public class HydrAuto_Specimen_Sweep extends HydrAuto_Specimen {

    private Pose2d mLastPosition;

    public HydrAuto_Specimen_Sweep() {
    }

    @Override
    protected SequentialAction SamplesToObsZone(Pose2d startPos) {

        Pose2d startSlideS2 = new Pose2d(-30, 40, HeadingRad(-135));
        Pose2d finishSlideS2 = new Pose2d(-30, 45, HeadingRad(-230));

        Pose2d startSlideS3 = new Pose2d(-40, 40, HeadingRad(-135));
        Pose2d finishSlideS3 = new Pose2d(-40, 45, HeadingRad(-240));

        Pose2d startSlideS4 = new Pose2d(-48, 40, HeadingRad(-135));
        Pose2d finishSlideS4 = new Pose2d(-50, 45, HeadingRad(-250));

        Action driveToS2 = mDrive.actionBuilder(startPos)
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

        mLastPosition = finishSlideS3;

        /*Action driveToS4 = mDrive.actionBuilder(finishSlideS3)
                .splineToLinearHeading(startSlideS4, HeadingRad(180))
                .build();

        Action pushS4ToObs = mDrive.actionBuilder(startSlideS4)
                .splineToLinearHeading(finishSlideS4, HeadingRad(-90))
                .build();

        mLastPosition = finishSlideS4;*/

        return new SequentialAction(
                // drive to the sample on the floor
                // bring the arm down to push
                new ParallelAction(
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
                pushS3ToObs
                /*new ParallelAction (
                        mArm.GetBasketPostScore(15, 0),
                        driveToS4
                ),
                mArm.GetAction(ArmActions.RunAutoSamplePush),
                pushS4ToObs,*/
        );
    }

    @Override
    protected Action PickupS2(Pose2d endPos) {
        return mDrive.actionBuilder(mLastPosition)
                .splineToLinearHeading(endPos, HeadingRad(0))
                .build();
    }
}
