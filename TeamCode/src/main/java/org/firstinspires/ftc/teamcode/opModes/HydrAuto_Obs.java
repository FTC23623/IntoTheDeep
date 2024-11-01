package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
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

@Autonomous(name="HydrAuto_Obs")
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
        mBeginPose = new Pose2d(-5.5, 63.5, HeadingRad(-90));
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
        Pose2d chamber = new Pose2d(-5.5, 42.25, HeadingRad(-90));

        Action takeS1ToChamber = mDrive.actionBuilder(mBeginPose)
                .splineToLinearHeading(chamber, HeadingRad(0))
                .build();

        Action park = mDrive.actionBuilder(chamber)
                .splineToLinearHeading(new Pose2d(-48, 12, HeadingRad(-90)), HeadingRad(0))
                .build();

        return new SequentialAction(
                mArm.GetAction(ArmActions.RunScoreHighOverBar),
                takeS1ToChamber,
                mArm.GetAction(ArmActions.RunScoreHighDropWrist),
                mArm.GetAction(ArmActions.RunScoreHighScore)
                //ScoreActions(),
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
               // mArm.GetAction(ArmActions.RunAscent1)
                //park
        );
    }

    public SequentialAction ScoreActions() {
        return new SequentialAction(
                mArm.GetAction(ArmActions.RunScoreHigh),
                //mArm.AdvanceScore(),
                new SleepAction(0.5),
                mArm.GetBasketPostScore(-10, 0.05),
                mArm.GetAction(ArmActions.RunCarry),
                mArm.GetAction(ArmActions.RunPickup),
                mArm.GetAction(ArmActions.RunHome)
        );
    }

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }
}
