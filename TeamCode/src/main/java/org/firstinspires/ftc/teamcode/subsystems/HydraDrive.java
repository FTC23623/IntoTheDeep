package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraDrive {
    protected boolean mGyroAssist = true;
    protected final DcMotorEx mMotDrFrLt;
    protected final DcMotorEx mMotDrFrRt;
    protected final DcMotorEx mMotDrBkLt;
    protected final DcMotorEx mMotDrBkRt;
    protected final HydraImu mImu;
    protected final String cfgFrLt = "MotDrFrLt";
    protected final String cfgFrRt = "MotDrFrRt";
    protected final String cfgBkLt = "MotDrBkLt";
    protected final String cfgBkRt = "MotDrBkRt";
    protected final double cWheelDiameter = 3.78;
    protected final double cWheelCircumference = cWheelDiameter * Math.PI;
    protected final double cCountsPerWheelRevolution = 537.6;
    protected final double cCountsPerInch = cCountsPerWheelRevolution / cWheelCircumference;
    protected final HydraOpMode mOp;
    protected final double cRampDownStartPercentage = 0.9;
    protected final double cRampLowPower = 0.3;
    protected final double cRampUpRate = 0.05;
    protected final double cRampDownRate = 0.05;
    protected final double cDriveBoosted = 1;
    protected final double cDriveNormal = 0.9;
    protected final double cDriveSlow = 0.5;
    protected double mRampDownStart;
    protected double mCurrentDrivePower;
    protected double mCurrentDriveMaxPower;
    protected double mCurrentDriveHeading;
    protected final boolean cResetEncodersBetweenDrives = true;
    public HydraDrive(HydraOpMode op) {
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
        mMotDrFrLt.setDirection(DcMotor.Direction.REVERSE);
        mMotDrBkLt.setDirection(DcMotor.Direction.REVERSE);
        mMotDrFrRt.setDirection(DcMotor.Direction.FORWARD);
        mMotDrBkRt.setDirection(DcMotor.Direction.FORWARD);
        // reset the encoders
        SetAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // we want to brake when we aren't applying power
        mMotDrFrLt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrBkLt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrFrRt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrBkRt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // create an IMU
        mImu = new HydraImu_navx(mOp);
    }

    /**
     * Start a drive action. This function does not check whether the last requested action has completed
     * Drive.Busy MUST be checked if it is necessary to do so
     * @param inDrive the distance to drive forward or backward
     * @param inStrafe the distance to strafe left or right
     * @param inRotate the amount to rotate in either direction
     */
    public void Start(double inDrive, double inStrafe, double inRotate, double headingDegrees) {
        double frontLeftCurrent = 0;
        double frontRightCurrent = 0;
        double backLeftCurrent = 0;
        double backRightCurrent = 0;
        SetAllMotorPower(0);
        if (cResetEncodersBetweenDrives) {
            // Clean up the last drive to prepare for the next one
            SetAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            frontLeftCurrent = mMotDrFrLt.getCurrentPosition();
            frontRightCurrent = mMotDrFrRt.getCurrentPosition();
            backLeftCurrent = mMotDrBkLt.getCurrentPosition();
            backRightCurrent = mMotDrBkRt.getCurrentPosition();
        }
        // Front left target position
        double frontLeftTarget = frontLeftCurrent + ((inDrive + inStrafe + inRotate) * cCountsPerInch);
        mMotDrFrLt.setTargetPosition((int)frontLeftTarget);
        // Rear left target position
        double rearLeftTarget = backLeftCurrent + ((inDrive - inStrafe + inRotate) * cCountsPerInch);
        mMotDrBkLt.setTargetPosition((int)rearLeftTarget);
        // Front right target position
        double frontRightTarget = frontRightCurrent + ((inDrive - inStrafe - inRotate) * cCountsPerInch);
        mMotDrFrRt.setTargetPosition((int)frontRightTarget);
        // Rear right target position
        double rearRightTarget = backRightCurrent + ((inDrive + inStrafe - inRotate) * cCountsPerInch);
        mMotDrBkRt.setTargetPosition((int)rearRightTarget);
        // Keep the desired heading for the end of the drive
        mCurrentDriveHeading = headingDegrees;
        // Get the total drive so we can calculate when to ramp the power down
        double totalDrive = Math.abs(frontLeftTarget) + Math.abs(rearLeftTarget) + Math.abs(frontRightTarget) + Math.abs(rearRightTarget);
        // Start ramping down when the error is under this value
        mRampDownStart = ((1 - cRampDownStartPercentage) * totalDrive);
        // Start at this power
        mCurrentDrivePower = cRampLowPower;
        // Ramp up to this power
        if (inRotate != 0) {
            mCurrentDriveMaxPower = cDriveSlow;
        } else {
            mCurrentDriveMaxPower = cDriveNormal;
        }
        // Run to position
        SetAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set power
        SetAllMotorPower(mCurrentDrivePower);
    }

    /**
     * Are we busy driving?
     * @return true if the motors have not reached position
     */
    public boolean Busy() {
        boolean ret = false;
        // Get the target position for each motor
        double FLmotTarget = mMotDrFrLt.getTargetPosition();
        double FRmotTarget = mMotDrFrRt.getTargetPosition();
        double BLmotTarget = mMotDrBkLt.getTargetPosition();
        double BRmotTarget = mMotDrBkRt.getTargetPosition();
        // Get the current position of each motor
        double FLmotPos = mMotDrFrLt.getCurrentPosition();
        double FRmotPos = mMotDrFrRt.getCurrentPosition();
        double BLmotPos = mMotDrBkLt.getCurrentPosition();
        double BRmotPos = mMotDrBkRt.getCurrentPosition();
        // Calculate the current error for each motor
        double errorBkLt = Math.abs(BLmotTarget - BLmotPos);
        double errorBkRt = Math.abs(BRmotTarget - BRmotPos);
        double errorFrLt = Math.abs(FLmotTarget - FLmotPos);
        double errorFrRt = Math.abs(FRmotTarget - FRmotPos);
        // Total error for all motors
        double totalError = errorBkLt + errorBkRt + errorFrLt + errorFrRt;
        // Ramp
        if (totalError < mRampDownStart) {
            // Ramp down at the end
            if (mCurrentDrivePower > cRampLowPower) {
                mCurrentDrivePower -= cRampDownRate;
                if (mCurrentDrivePower < cRampLowPower) {
                    mCurrentDrivePower = cRampLowPower;
                }
                SetAllMotorPower(mCurrentDrivePower);
            }
        }
        else if (mCurrentDrivePower < mCurrentDriveMaxPower) {
            // Ramp up at the beginning
            mCurrentDrivePower += cRampUpRate;
            if (mCurrentDrivePower > mCurrentDriveMaxPower) {
                mCurrentDrivePower = mCurrentDriveMaxPower;
            }
            SetAllMotorPower(mCurrentDrivePower);

        }
        double yawError = 0;
        // the best thing to do is nothing if we have some bad code path
        double yaw = mCurrentDriveHeading;;
        boolean useImu = false;
        if (mGyroAssist) {
            if (!ImuConnected()) {
                mOp.mTelemetry.addData("Yaw", "disconnected");
            }
            else if (ImuCalibrating()) {
                mOp.mTelemetry.addData("Yaw", "cal");
            }
            else {
                useImu = true;
                yaw = mImu.GetYaw();
                // Track the error against our desired heading
                yawError = yaw - mCurrentDriveHeading;
                mOp.mTelemetry.addData("Yaw", yaw);
                mOp.mTelemetry.addData("YawError", yawError);
            }
        }
        if (mOp.mDriveLogger != null) {
            mOp.mDriveLogger.blposition.set(BLmotPos);
            mOp.mDriveLogger.bltarget.set(BLmotTarget);
            mOp.mDriveLogger.brposition.set(BRmotPos);
            mOp.mDriveLogger.brtarget.set(BRmotTarget);
            mOp.mDriveLogger.flposition.set(FLmotPos);
            mOp.mDriveLogger.fltarget.set(FLmotTarget);
            mOp.mDriveLogger.frposition.set(FRmotPos);
            mOp.mDriveLogger.frtarget.set(FRmotTarget);
            mOp.mDriveLogger.drMotPwr.set(mCurrentDrivePower * 100);
            for (VoltageSensor battvolt : mOp.mHardwareMap.voltageSensor) {
                // I assume it's the first battery voltage
                mOp.mDriveLogger.battVoltage.set(battvolt.getVoltage());
                break;
            }
            mOp.mDriveLogger.yawError.set(yawError);
            mOp.mDriveLogger.writeLine();
        }
        // if any motor is still active, we are still busy
        if (mMotDrBkLt.isBusy() || mMotDrBkRt.isBusy() || mMotDrFrLt.isBusy() || mMotDrFrRt.isBusy()) {
            ret = true;
        }
        else if (useImu) {
            if (Math.abs(yawError) > 2) {
                // 20 "inches" seems to be about 90 degrees
                double rotation = yawError * 20 / 90;
                Start(0, 0, rotation, mCurrentDriveHeading);
                ret = true;
            }
            else {
                ret = false;
            }
        }
        mOp.mTelemetry.addData("Driving", ret);
        mOp.mTelemetry.addData("Heading", mCurrentDriveHeading);
        return ret;
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

    /**
     * Sets the power to all motors
     * @param value the power to set to the motors
     */
    private void SetAllMotorPower(double value) {
        mMotDrBkLt.setPower(value);
        mMotDrBkRt.setPower(value);
        mMotDrFrLt.setPower(value);
        mMotDrFrRt.setPower(value);
    }

    /**
     * Sets the run mode for all motors
     * @param value the run mode to set
     */
    private void SetAllMotorMode(DcMotor.RunMode value) {
        mMotDrBkLt.setMode(value);
        mMotDrBkRt.setMode(value);
        mMotDrFrLt.setMode(value);
        mMotDrFrRt.setMode(value);
    }
}
