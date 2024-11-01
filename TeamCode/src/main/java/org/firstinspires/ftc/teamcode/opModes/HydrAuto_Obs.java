package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_navx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

@Autonomous(name="HydrAuto_Obs", preselectTeleOp = "HyDrive_Specimen")
public class HydrAuto_Obs extends LinearOpMode {
    protected HydraOpMode mOpMode;
    protected MecanumDrive mDrive;
    protected Imu mImu;
    protected Arm mArm;
    protected Intake mIntake;
    protected ElementTypes mElementType;
    protected Pose2d mBeginPose;

    @Override
    public void runOpMode() throws InterruptedException {
        mElementType = ElementTypes.Specimen;
        mBeginPose = new Pose2d(-2.5, 63.5, HeadingRad(-90));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null,
                null, null, mElementType);
        mImu = new Imu_navx(mOpMode);
        mArm = new Arm(mOpMode, true);
        mIntake = new Intake(mOpMode);
        mDrive = new MecanumDrive(hardwareMap, mBeginPose);
        while (!mImu.Connected() || mImu.Calibrating()) {
            if (isStopRequested() || !opModeIsActive()) {
                break;
            }
        }
        SequentialAction autoSeq = CreateAutoSeq();
        waitForStart();
        while (opModeIsActive()) {
            mIntake.RunIn();
            mIntake.Process();
            if (mArm.Startup(false)) {
                break;
            }
            idle();
        }
        mIntake.Stop();
        mIntake.Process();
        Actions.runBlocking(autoSeq);
        while (opModeIsActive());
        OpmodeHeading.SetOffset(mImu.GetYaw());
        OpmodeHeading.handOff = true;
    }

    public SequentialAction CreateAutoSeq() {
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

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }
}
