package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

@Autonomous(name="HydrAuto_Obs", preselectTeleOp = "HyDrive_Specimen")
public class HydrAuto_Obs extends HydrAuto {

    public HydrAuto_Obs() {
        mElementType = ElementTypes.Specimen;
        mBeginPose = new Pose2d(-2.5, 63.5, HeadingRad(-90));
    }

    @Override
    protected SequentialAction CreateAuto() {
        Pose2d chamberPos = new Pose2d(-2.5, 44, HeadingRad(-90));
        Pose2d forwardPos = new Pose2d(-2.5, 41, HeadingRad(-90));
        Pose2d afterScorePos = new Pose2d(-2.5, 45, HeadingRad(-90));
        Pose2d parkPos = new Pose2d(-48, 60, HeadingRad(-90));

        Action takeS1ToChamber = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(chamberPos, HeadingRad(-90))
                .build();

        Action forward = mDrive.actionBuilder(chamberPos)
                .splineToLinearHeading(forwardPos, HeadingRad(-90))
                .build();

        Action park = mDrive.actionBuilder(forwardPos)
                .splineToLinearHeading(afterScorePos, HeadingRad(90))
                .splineToLinearHeading(parkPos, HeadingRad(180))
                .build();

        return new SequentialAction(
                mArm.GetAction(ArmActions.RunScoreHighOverBar),
                takeS1ToChamber,
                new SleepAction(1),
                mArm.GetAction(ArmActions.RunScoreHighDropWrist),
                new SleepAction(0.5),
                forward,
                mIntake.GetAction(IntakeActions.InStart),
                new SleepAction(0.1),
                mArm.GetAction(ArmActions.RunScoreHighScore),
                mArm.GetAction(ArmActions.RunPickup),
                mIntake.GetAction(IntakeActions.Stop),
                park,
                mArm.GetAction(ArmActions.RunHome)
        );
    }
}
