package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

@Config
public class Arm {
    private HydraOpMode mOp;
    private PIDController mPID;
    private DcMotor mLiftMotor;
    private DcMotor mSlideMotor;
    // Lift arm PIDF controller gains
    private final double mLiftP = 0.004;
    private final double mLiftI = 0.0001;
    private final double mLiftD = 0.0001;
    private final double mLiftF = 0.1;
    // Lift arm motor ticks per degree
    // 1993.6 PPR at the motor
    // 2x1 gear
    private final double mLiftTicksPerDegree = 1993.6 / 180.0;
    // Lift arm desired position in ticks
    private int mLiftPositionTicks;
    // Lift arm desired position in degrees
    private double mLiftPositionDeg;
    // Lift arm motor zero position (home)
    public static double mLiftZeroPosDeg = -15.0;
    // Lift arm motor max lift position
    public static double mLiftMaxPosDeg = 115.0;
    // Slide motor desired position in ticks
    public static int mArmExtendTicks;
    // Slide extension in inches
    public static double mArmExtendInches;
    // 1993.6 PPR at the motor
    // inches / revolution?
    private final double mArmExtendTicksPerInch = 1993.6 / ;
    // Max extension of the arm in inches
    private final double mArmExtendMaxInches = 40.0;
    // Base length of the arm, including the distance we want to keep from the floor (on an angle)
    private final double mArmBaseLenInches = ;
    // Height of the arm pivot from the floor
    private final double mArmPivotHeightInches = ;
    // Power level for slide motor
    private final double mSlideMotorPower = 1.0;
    // Whether or not we're currently utilizing manual extension to pick up samples
    private boolean mManualExtension;

    /**
     * Initializes the Arm object
     * Lift and extension
     * @param opMode contains various classes needed for subsystems used with an opmode
     */
    public Arm(HydraOpMode opMode) {
        mOp = opMode;
        mLiftMotor = opMode.mHardwareMap.get(DcMotorEx.class, "liftMotor");
        mSlideMotor = opMode.mHardwareMap.get(DcMotorEx.class, "slideMotor");
        mPID = new PIDController(mLiftP, mLiftI, mLiftD);
        mLiftPositionTicks = 0;
        mLiftPositionDeg = mLiftZeroPosDeg;
        mLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mArmExtendTicks = 0;
        mArmExtendInches = 0;
        mSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mManualExtension = false;
    }

    /**
     * Set the desired arm lift position
     * @param degrees: absolute position of the arm in degrees
     */
    public void SetLiftArmAngle(double degrees) {
        // check the bounds of the input and cap the angle
        if (degrees < mLiftZeroPosDeg) {
            degrees = mLiftZeroPosDeg;
        } else if (degrees > mLiftMaxPosDeg) {
            degrees = mLiftMaxPosDeg;
        }
        // subtract the zero position for the motors before we go to ticks
        double degreesFromZeroPosition = degrees - mLiftZeroPosDeg;
        // convert degrees to motor ticks
        mLiftPositionTicks = (int)(degreesFromZeroPosition * mLiftTicksPerDegree);
        // keep this around for later math
        mLiftPositionDeg = degrees;
    }

    /**
     * Set the desired extension of the arm
     * @param inches: length of the arm extension
     */
    public void SetArmExtension(double inches) {
        // check the bounds of the input
        if (inches < 0) {
            inches = 0;
        } else if (inches > mArmExtendMaxInches) {
            inches = mArmExtendMaxInches;
        }
        // convert inches to motor ticks
        mArmExtendTicks = (int)(inches * mArmExtendTicksPerInch);
        mArmExtendInches = inches;
    }

    /**
     * Call from main loop to handle user input in teleop
     * This is not used for auto opmodes
     */
    public void HandleUserInput() {
        // The arm lift positions are set by the d-pad(?)
        // we only want to accept user commands for this if the slides are fully retracted
        if ( /* fill in logic to avoid changing arm angle while extended */ ) {
            // set the up position for the arm
            if ( /* button for lifting arm up */ ) {

            } else if ( /* button for moving arm home */) {

            }
        }
        // The extension is manually controlled by the left thumbstick
        // use HydraConstants.joyStickDeadBand
        mManualExtension = false;
        if ( /* check joystick against HydraConstants.joyStickDeadBand */ ) {
            /* set power for the motor based on the joystick, bound by 0 and max position */
            // automatically raise the arm while we extend to keep from crashing into the floor
            mManualExtension = true;
        } else if ( /* check button that we will use to extend the arm to the bucket */ ) {

        }
    }

    /**
     * Uses PID controller to keep the arm at the desired set position
     * Uses run-to-position to actuate the linear slides for arm extension
     */
    public void Process() {
        // Get the current slide position to compare
        int currentSlide = mSlideMotor.getCurrentPosition();
        // If we're close then we don't need to do anything
        if (Math.abs(currentSlide - mArmExtendTicks) > 5) {
            mSlideMotor.setTargetPosition(mArmExtendTicks);
            mSlideMotor.setPower(mSlideMotorPower);
            mSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            mSlideMotor.setPower(0);
        }
        // Telemetry for debugging
        mOp.mTelemetry.addData("Slide Pos", currentSlide);
        mOp.mTelemetry.addData("Slide Target", mArmExtendTicks);
        if (mManualExtension) {
            // override the set angle and set based on the extension
            // hypotenuse is the arm
            // adjacent is the vertical we are mounted to (extended to the floor)
            // use cos to get the set angle
            SetLiftArmAngle(Math.acos((mArmBaseLenInches + mArmExtendInches) / mArmPivotHeightInches));
        }
        // Get current position for calculations
        int currentPos = mLiftMotor.getCurrentPosition();
        // Calculate the pid to get from current position to desired
        double pid = mPID.calculate(currentPos, mLiftPositionTicks);
        // Factor in gravity. Cos will scale appropriately based on the angle
        double ff = Math.cos(Math.toRadians(mLiftPositionDeg)) * mLiftF;
        // Add pid and feed forward to get the final power
        double power = pid + ff;
        // Set power to the motor
        mLiftMotor.setPower(power);
        // Telemetry for debugging
        mOp.mTelemetry.addData("Lift Pos", currentPos);
        mOp.mTelemetry.addData("Lift Target", mLiftPositionTicks);
        mOp.mTelemetry.addData("Lift Power", power);
    }

    public boolean LiftBusy() {
        return mLiftMotor.isBusy();
    }

    public boolean ExtendBusy() {
        return mSlideMotor.isBusy();
    }
}
