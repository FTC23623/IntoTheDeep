package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_navx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArm;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ElementTypes;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

import java.util.List;

public class HydrAuto extends LinearOpMode {
    protected HydraOpMode mOpMode;
    protected MecanumDrive mDrive;
    protected Imu mImu;
    protected Arm mArm;
    protected SpecimenArm mSpecArm;
    protected Claw mClaw;
    protected Intake mIntake;
    protected ElementTypes mElementType;
    protected Pose2d mBeginPose;
    protected ElapsedTime mTimeSinceStart;
    protected SequentialAction mAutoSeq;
    protected boolean mRunIntakeAtStart;

    @Override
    public void runOpMode() throws InterruptedException {
        OpmodeHeading.handOff = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null,
                null, null, mElementType);
        mImu = new Imu_navx(mOpMode);
        mArm = new Arm(mOpMode);
        mSpecArm = new SpecimenArm(mOpMode);
        mClaw = new Claw(mOpMode);
        mIntake = new Intake(mOpMode);
        mDrive = new MecanumDrive(hardwareMap, mBeginPose);
        mTimeSinceStart = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (!mImu.Connected() || mImu.Calibrating()) {
            if (isStopRequested() || !opModeIsActive()) {
                break;
            }
        }
        mAutoSeq = CreateAuto();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();
        mTimeSinceStart.reset();
        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            if (mRunIntakeAtStart) {
                mIntake.RunIn();
                mIntake.Process();
            }
            if (mArm.Startup(false) && (!mRunIntakeAtStart || (mTimeSinceStart.milliseconds() > 250))) {
                break;
            }
            idle();
        }
        if (mRunIntakeAtStart) {
            mIntake.Stop();
        }
        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            mArm.Process();
            mIntake.Process();
            mSpecArm.Process();
            if(!mAutoSeq.run(packet)) {
                break;
            }
            telemetry.addLine(packet.toString());
            telemetry.update();
            idle();
        }
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        OpmodeHeading.SetOffset(mImu.GetYaw());
        OpmodeHeading.handOff = true;
    }

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }

    protected SequentialAction CreateAuto() {
        return null;
    }
}
