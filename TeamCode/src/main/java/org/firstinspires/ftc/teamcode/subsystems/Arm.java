package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class Arm {
    private HydraOpMode mOp;
    private PIDController mPID;
    public static double mLiftP;
    public static double mLiftI;
    public static double mLiftD;
    public static double mLiftF;
    // 1993.6 PPR at the motor
    // 2x1 gear
    private final double mLiftMotorTicksInDeg = 1993.6 / 180.0;
    public static int mLiftPosition;
    private DcMotor mLiftMotor;
    private DcMotor mSlideMotor;

    public Arm(HydraOpMode opMode) {
        mOp = opMode;
        mLiftMotor = opMode.mHardwareMap.get(DcMotorEx.class, "liftMotor");
        mSlideMotor = opMode.mHardwareMap.get(DcMotorEx.class, "slideMotor");
        mLiftP = 0;
        mLiftI = 0;
        mLiftD = 0;
        mLiftF = 0;
        mPID = new PIDController(mLiftP, mLiftI, mLiftD);
        mLiftPosition = 0;
    }

    public void Process() {
        mPID.setPID(mLiftP, mLiftI, mLiftD);
        int currentPos = mLiftMotor.getCurrentPosition();
        double pid = mPID.calculate(currentPos, mLiftPosition);
        double ff = Math.cos(Math.toRadians(mLiftPosition / mLiftMotorTicksInDeg)) * mLiftF;
        double power = pid * ff;
        mLiftMotor.setPower(power);
        mOp.mTelemetry.addData("Lift Pos", currentPos);
        mOp.mTelemetry.addData("Lift Target", mLiftPosition);
    }
}
