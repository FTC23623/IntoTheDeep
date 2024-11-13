package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

@Config
public class SpecimenArm {
    private HydraOpMode mOp;
    private final DcMotorEx mLiftMotor;
    private double mLiftTargetDeg;
    private final double mTicksPerDegree = 288.0 / 360.0;
    
    public SpecimenArm(HydraOpMode opMode) {
        mOp = opMode;
        mLiftMotor = mOp.mHardwareMap.get(DcMotorEx.class, "specArmMotor");
        mLiftTargetDeg = 0;
        //mLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //mLiftMotor.setTargetPosition(0);
        //mLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //mLiftMotor.setPower(0.5);
    }

    public void SetAngle(double angle) {
        mLiftTargetDeg = angle;
    }

    public void Process() {
        mLiftMotor.setTargetPosition((int)(mLiftTargetDeg * mTicksPerDegree));
        mLiftMotor.setPower(0.5);
        mOp.mTelemetry.addData("A", mLiftTargetDeg * mTicksPerDegree);
        mOp.mTelemetry.addData("A Tgt", mLiftMotor.getTargetPosition());
        mOp.mTelemetry.addData("A Pos", mLiftMotor.getCurrentPosition());
    }
}
