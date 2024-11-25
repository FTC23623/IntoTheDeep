package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="HydrAuto_Specimen_Push", preselectTeleOp = "HyDrive_Specimen")
public class HydrAuto_Specimen_Push extends HydrAuto_Specimen {

    private Pose2d mLastPosition;

    public HydrAuto_Specimen_Push() {

    }

    @Override
    protected SequentialAction SamplesToObsZone(Pose2d startPos) {
        Pose2d startPushS2 = new Pose2d(-46, 12, HeadingRad(-90));
        Pose2d finishPushS2 = new Pose2d(-46, 60, HeadingRad(-90));

        Pose2d startPushS3 = new Pose2d(-56, 12, HeadingRad(-90));
        Pose2d finishPushS3 = new Pose2d(-56, 60, HeadingRad(-90));

        mLastPosition = finishPushS3;

        /*Pose2d startPushS4 = new Pose2d(-66, 12, HeadingRad(-90));
        Pose2d finishPushS4 = new Pose2d(-66, 60, HeadingRad(-90));

        mLastPosition = finishPushS4;*/

        return new SequentialAction(mDrive.actionBuilder(startPos)
                .setTangent(HeadingRad(180))
                .splineToLinearHeading(startPushS2, HeadingRad(180))
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(finishPushS2, HeadingRad(-90))
                .setTangent(HeadingRad(-90))
                .splineToLinearHeading(startPushS3, HeadingRad(180))
                .setTangent(HeadingRad(90))
                .splineToLinearHeading(finishPushS3, HeadingRad(-90))
                .build()
        );
    }

    @Override
    protected Action PickupS2(Pose2d endPos) {
        return mDrive.actionBuilder(mLastPosition)
                .splineToLinearHeading(endPos, HeadingRad(0))
                .build();
    }
}
