package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_navx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

public abstract class HydrAuto extends LinearOpMode {
    protected HydraOpMode mOpMode;
    protected MecanumDrive mDrive;
    protected Imu mImu;
    protected Arm mArm;
    protected Intake mIntake;
    protected ElementTypes mElementType;
    protected Pose2d mBeginPose;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null,
                null, null, mElementType);
        mImu = new Imu_navx(mOpMode);
        mArm = new Arm(mOpMode);
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
            if (mArm.Startup(false)) {
                break;
            }
            idle();
        }
        Actions.runBlocking(autoSeq);
    }

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }

    public abstract SequentialAction CreateAutoSeq();
}
