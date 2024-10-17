package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

@Config
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
    public static int mSlidePosition;
    public static double mSlideMotorPower;
    private DcMotor mSlideMotor;

    public Arm(HydraOpMode opMode) {
        mOp = opMode;
        mLiftMotor = opMode.mHardwareMap.get(DcMotorEx.class, "liftMotor");
        mSlideMotor = opMode.mHardwareMap.get(DcMotorEx.class, "slideMotor");
        mLiftP = 0.004;
        mLiftI = 0.0001;
        mLiftD = 0.0001;
        mLiftF = 0.1;
        mPID = new PIDController(mLiftP, mLiftI, mLiftD);
        mLiftPosition = 0;
        mLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlidePosition=0;
        mSlideMotorPower=0.5;
        mSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void Test(double p, double i, double d, double f, int pos) {
        mLiftP = p;
        mLiftI = i;
        mLiftD = d;
        mLiftF = f;
        mLiftPosition = pos;
    }

    public void Process() {
        mPID.setPID(mLiftP, mLiftI, mLiftD);
        int currentPos = mLiftMotor.getCurrentPosition();
        double pid = mPID.calculate(currentPos, mLiftPosition);
        double ff = Math.cos(Math.toRadians(mLiftPosition / mLiftMotorTicksInDeg)) * mLiftF;
        double power = pid + ff;
        mLiftMotor.setPower(power);
        mOp.mTelemetry.addData("Lift Pos", currentPos);
        mOp.mTelemetry.addData("Lift Target", mLiftPosition);
        mOp.mTelemetry.addData("Power", power);
        mOp.mTelemetry.addData("P", mLiftP);
        mOp.mTelemetry.addData("I", mLiftI);
        mOp.mTelemetry.addData("D", mLiftD);
        mOp.mTelemetry.addData("F", mLiftF);
        mOp.mTelemetry.addData("pid", pid);
        mOp.mTelemetry.addData("ff", ff);
        int currentSlide = mSlideMotor.getCurrentPosition();
        if (Math.abs(currentSlide - mSlidePosition) > 5) {
            mSlideMotor.setTargetPosition(mSlidePosition);
            mSlideMotor.setPower(mSlideMotorPower);
            mSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            mSlideMotor.setPower(0);
        }
    }
}
