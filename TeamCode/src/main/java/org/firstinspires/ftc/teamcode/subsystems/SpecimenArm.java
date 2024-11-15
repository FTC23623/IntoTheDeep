package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

@Config
public class SpecimenArm {
    private HydraOpMode mOp;
    private final DcMotorEx mLiftMotor;
    private final PIDController mSpecArmPid;
    // PID coefficients
    public static double mSpecArmP = 0.022;
    public static double mSpecArmI = 0.0;
    public static double mSpecArmD = 0.0018;
    // Target in degrees
    private double mLiftTargetTicks;
    // REV Core Hex
    // 288 ticks per rotation
    private final double mTicksPerDegree = 288.0 / 360.0;
    public static double mStartAngle = 8.0;
    // under at 117
    // score run to 35
    
    public SpecimenArm(HydraOpMode opMode) {
        mOp = opMode;
        mLiftMotor = mOp.mHardwareMap.get(DcMotorEx.class, "specArmMotor");
        mSpecArmPid = new PIDController(mSpecArmP, mSpecArmI, mSpecArmD);
        mLiftTargetTicks = 0;
        mLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void SetAngle(double angle) {
        mLiftTargetTicks = (angle - mStartAngle) * mTicksPerDegree;
    }

    public void Process() {
        // get the current position to calculate error
        int currentPos = mLiftMotor.getCurrentPosition();
        mSpecArmPid.setPID(mSpecArmP, mSpecArmI, mSpecArmD);
        // calculate pid for power and run
        double pid = mSpecArmPid.calculate(currentPos, mLiftTargetTicks);
        mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLiftMotor.setPower(pid);
        mOp.mTelemetry.addData("A Tgt", mLiftTargetTicks);
        mOp.mTelemetry.addData("A Pos", mLiftMotor.getCurrentPosition());
        mOp.mTelemetry.addData("A Pwr", pid);
    }
}
