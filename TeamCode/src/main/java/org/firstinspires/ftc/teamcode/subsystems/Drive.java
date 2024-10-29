package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class Drive {
    protected boolean mGyroAssist = true;
    protected final DcMotorEx mMotDrFrLt;
    protected final DcMotorEx mMotDrFrRt;
    protected final DcMotorEx mMotDrBkLt;
    protected final DcMotorEx mMotDrBkRt;
    protected final Imu mImu;
    protected final String cfgFrLt = "leftFront";
    protected final String cfgFrRt = "rightFront";
    protected final String cfgBkLt = "leftBack";
    protected final String cfgBkRt = "rightBack";
    protected final double cWheelDiameter = 3.78;
    protected final double cWheelCircumference = cWheelDiameter * Math.PI;
    protected final double cCountsPerWheelRevolution = 537.6;
    protected final double cCountsPerInch = cCountsPerWheelRevolution / cWheelCircumference;
    protected final HydraOpMode mOp;
    protected final double cRampDownStartPercentage = 0.9;
    protected final double cRampLowPower = 0.3;
    protected final double cRampUpRate = 0.05;
    protected final double cRampDownRate = 0.05;
    protected double mRampDownStart;
    protected double mCurrentDrivePower;
    protected double mCurrentDriveMaxPower;
    protected double mCurrentDriveHeading;
    protected final boolean cResetEncodersBetweenDrives = true;
    public Drive(HydraOpMode op, Imu imu) {
        mOp = op;
        // grab the motors out of the hardware map
        mMotDrFrLt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, cfgFrLt);
        mMotDrFrRt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, cfgFrRt);
        mMotDrBkLt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, cfgBkLt);
        mMotDrBkRt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, cfgBkRt);
        if (false) {
            // todo try setting new PID values
            PIDFCoefficients pid = mMotDrFrLt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            mOp.mTelemetry.addData("FrLft PID", pid.toString());
            pid = mMotDrFrRt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            mOp.mTelemetry.addData("FrRt PID", pid.toString());
            pid = mMotDrBkLt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            mOp.mTelemetry.addData("BkLft PID", pid.toString());
            pid = mMotDrBkRt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            mOp.mTelemetry.addData("BkRt PID", pid.toString());
            PIDFCoefficients newPIDF = new PIDFCoefficients(pid.p, pid.i, pid.d, pid.f, MotorControlAlgorithm.PIDF);
            mMotDrBkLt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
            mMotDrBkRt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
            mMotDrFrLt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
            mMotDrFrRt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
        }
        // increase the tolerance in the PID
        mMotDrBkLt.setTargetPositionTolerance(10);
        mMotDrBkRt.setTargetPositionTolerance(10);
        mMotDrFrLt.setTargetPositionTolerance(10);
        mMotDrFrRt.setTargetPositionTolerance(10);
        // set the motor directions
        mMotDrFrLt.setDirection(DcMotor.Direction.FORWARD);
        mMotDrBkLt.setDirection(DcMotor.Direction.FORWARD);
        mMotDrFrRt.setDirection(DcMotor.Direction.REVERSE);
        mMotDrBkRt.setDirection(DcMotor.Direction.REVERSE);
        // reset the encoders
        SetAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // we want to brake when we aren't applying power
        mMotDrFrLt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrBkLt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrFrRt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrBkRt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // create an IMU
        mImu = imu;
    }

    protected boolean ImuCalibrating() {
        return mImu.Calibrating();
    }

    protected boolean ImuConnected() {
        return mImu.Connected();
    }

    public boolean ImuReady() {
        return ImuConnected() && !ImuCalibrating();
    }

    public void CloseImu() {
        mImu.Close();
    }

    public double GetYaw() {
        if (ImuReady()) {
            return mImu.GetYaw();
        }
        return 0;
    }

    public void Process() {

    }

    /**
     * Sets the power to all motors
     * @param value the power to set to the motors
     */
    protected void SetAllMotorPower(double value) {
        mMotDrBkLt.setPower(value);
        mMotDrBkRt.setPower(value);
        mMotDrFrLt.setPower(value);
        mMotDrFrRt.setPower(value);
    }

    /**
     * Sets the run mode for all motors
     * @param value the run mode to set
     */
    protected void SetAllMotorMode(DcMotor.RunMode value) {
        mMotDrBkLt.setMode(value);
        mMotDrBkRt.setMode(value);
        mMotDrFrLt.setMode(value);
        mMotDrFrRt.setMode(value);
    }
}
