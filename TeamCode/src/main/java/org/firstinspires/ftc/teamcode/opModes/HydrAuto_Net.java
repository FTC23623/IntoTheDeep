package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Autonomous(name = "HydrAuto_Net", preselectTeleOp = "HyDrive_Sample")
public class HydrAuto_Net extends LinearOpMode {
    protected HydraOpMode mOpMode;
    protected MecanumDrive mDrive;
    protected Imu mImu;
    protected Arm mArm;
    protected Intake mIntake;
    protected ElementTypes mElementType;
    protected Pose2d mBeginPose;

    @Override
    public void runOpMode() throws InterruptedException {
        mElementType = ElementTypes.Sample;
        mBeginPose = new Pose2d(39.5, 63.5, HeadingRad(-90));
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
        OpmodeHeading.SetOffset(mImu.GetYaw());
        OpmodeHeading.handOff = true;
    }

    public SequentialAction CreateAutoSeq() {
        Pose2d basket = new Pose2d(52, 47, HeadingRad(-135));
        Pose2d s2 = new Pose2d(59, 46, HeadingRad(-90));
        Pose2d s3 = new Pose2d(49, 46, HeadingRad(-90));
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
                /*
                new ParallelAction(
                        driveToS2,
                        mIntake.GetAction(IntakeActions.InStart)
                ),
                takeS2ToBasket,
                ScoreActions(),
                new ParallelAction(
                        driveToS3,
                        mIntake.GetAction(IntakeActions.InStart)
                ),
                takeS3ToBasket,
                ScoreActions(),
                new ParallelAction(
                        driveToS4,
                        mIntake.GetAction(IntakeActions.InStart)
                ),
                takeS4ToBasket,
                ScoreActions(),
                 */
                //mArm.GetAction(ArmActions.RunAscent1),
                park
        );
    }

    public SequentialAction ScoreActions() {
        return new SequentialAction(
                mArm.GetAction(ArmActions.RunScoreHigh),
                mIntake.GetAction(IntakeActions.OutContinuous),
                new SleepAction(0.5),
                mArm.GetBasketPostScore(-10, 0.05),
                mArm.GetAction(ArmActions.RunCarry),
                mArm.GetAction(ArmActions.RunPickup)
                //mArm.GetAction(ArmActions.RunHome)
        );
    }

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }
}
