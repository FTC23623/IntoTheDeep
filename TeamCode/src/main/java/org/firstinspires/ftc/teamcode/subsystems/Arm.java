package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ArmMoveStates;
import org.firstinspires.ftc.teamcode.types.ArmPositions;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Config
public class Arm {
    private HydraOpMode mOp;
    private PIDController mPID;
    private DcMotor mLiftMotor;
    private DcMotor mSlideMotor;
    private Servo mWristServo;
    private RevTouchSensor mExtendHomeSwitch;
    private RevTouchSensor mLiftHomeSwitch;
    private com.qualcomm.robotcore.hardware.Gamepad mControl;
    // Lift arm PIDF controller gains
    private final double mLiftP = 0.004;
    private final double mLiftI = 0.0009;
    private final double mLiftD = 0.0001;
    private final double mLiftFRetracted = 0.18;
    // TODO: make private
    public static double mLiftFExtended = 0.20;
    // Lift arm motor ticks per degree
    // 1993.6 PPR at the motor
    // 2x1 gear
    // TODO: verify this value
    private final double mLiftTicksPerDegree = 1993.6 / 180.0;
    // Lift arm desired position in ticks
    private int mLiftPositionTicks;
    // Lift arm desired position in degrees
    private double mLiftPositionDeg;
    // Lift arm motor zero position (home)
    // TODO: verify this value
    private final double mLiftZeroPosDeg = -20.0;
    // Lift arm motor max lift position
    // TODO: verify this value
    private final double mLiftMaxPosDeg = 99.0;
    // Lift motor boundaries for auto wrist angles
    private final double mLiftBoundaryFloorDeg = 0.0;
    private final double mLiftBoundaryChamberDeg = 80.0;
    // Slide motor desired position in ticks
    private int mArmExtendTicks;
    // Slide extension in inches
    private double mArmExtendInches;
    // 1993.6 PPR at the motor
    // inches / revolution?
    // TODO: verify this value
    private final double mArmExtendTicksPerInch = 384.5 / 4.72441;
    // Max extension of the arm in inches
    // TODO: verify this value
    // max physically possible 27.95 inches
    private final double mArmExtendMaxInches = 18.5;
    // Base length of the arm, including the distance we want to keep from the floor (on an angle)
    // TODO: verify this value
    private final double mArmBaseLenInches = 14.7;
    // Height of the bottom of the arm from the floor
    // TODO: verify this value
    private final double mArmPivotHeightInches = 7.76;
    // Power level for slide motor
    private final double mSlideMotorPower = 1.0;
    // Whether or not we're currently utilizing manual extension to pick up samples
    private boolean mManualMode;
    // max position value for the wrist servo
    private final double mWristServoMaxPos = 1.0;
    // min position value for the wrist servo
    private final double mWristServoMinPos = -0.5;
    // min time in seconds for full traversing of wrist servo range
    private final double mWristServoTravelTime = 0.5;
    // how much to increment the wrist servo position by every time the loop runs
    private double mWristServoManualIncrement;
    // manual controller input for the wrist servo
    private double mManualWristInput;
    // position to set for the wrist servo
    private double mServoPosition;
    private final boolean mEnableTune = true;
    private ArmActions mAction;
    private ArmActions mLastActiveAction;
    // predetermined lift and extend positions for all arm positions
    private final double Pos0Home_Lift = mLiftZeroPosDeg;
    private final double Pos0Home_Extend = 0.0;
    private final double Pos0Home_Wrist = 1.0;
    private final double Pos1ManualPickup_Lift = -10.0;
    private final double Pos1ManualPickup_Extend = 0.0;
    private final double Pos1ManualPickup_Wrist = -0.3;
    private final double Pos2FloorPickup_Lift = mLiftZeroPosDeg;
    private final double Pos2FloorPickup_Extend = 0.0;
    private final double Pos2FloorPickup_Wrist = -0.3;
    private final double Pos3SpecimenPickup_Lift = -10.0;
    private final double Pos3SpecimenPickup_Extend = 0.0;
    private final double Pos3SpecimenPickup_Wrist = 0.0;
    private final double Pos4SpecimenLowerChamber_Lift = 5.0;
    private final double Pos4SpecimenLowerChamber_Extend = 0.0;
    private final double Pos4SpecimenLowerChamber_Wrist = 0.3;
    private final double Pos5SpecimenUpperChamber_Lift = 30.0;
    private final double Pos5SpecimenUpperChamber_Extend = 6.0;
    private final double Pos5SpecimenUpperChamber_Wrist = 0.3;
    private final double Pos6SampleLowerBasket_Lift = 95.0;
    private final double Pos6SampleLowerBasket_Extend = 6.0;
    private final double Pos6SampleLowerBasket_Wrist = -0.1;
    private final double Pos7SampleUpperBasket_Lift = 99.0;
    private final double Pos7SampleUpperBasket_Extend = 18.5;
    private final double Pos7SampleUpperBasket_Wrist = -0.1;
    // create arrays with the preset values for quick lookup
    double mLiftPositions[] = { Pos0Home_Lift, Pos1ManualPickup_Lift, Pos2FloorPickup_Lift, Pos3SpecimenPickup_Lift,
            Pos4SpecimenLowerChamber_Lift, Pos5SpecimenUpperChamber_Lift, Pos6SampleLowerBasket_Lift, Pos7SampleUpperBasket_Lift };
    double mExtendPositions[] = { Pos0Home_Extend, Pos1ManualPickup_Extend, Pos2FloorPickup_Extend, Pos3SpecimenPickup_Extend,
            Pos4SpecimenLowerChamber_Extend, Pos5SpecimenUpperChamber_Extend, Pos6SampleLowerBasket_Extend, Pos7SampleUpperBasket_Extend };
    double mWristPositions[] = { Pos0Home_Wrist, Pos1ManualPickup_Wrist, Pos2FloorPickup_Wrist, Pos3SpecimenPickup_Wrist,
            Pos4SpecimenLowerChamber_Wrist, Pos5SpecimenUpperChamber_Wrist, Pos6SampleLowerBasket_Wrist, Pos7SampleUpperBasket_Wrist };
    // index into the position arrays for current movement
    private int mArmPosIdx;
    // current state of an arm movement
    private ArmMoveStates mMoveState;
    // user supplied extension power
    private double mManualExtendInput;

    /**
     * Initializes the Arm object
     * Lift and extension
     * @param opMode contains various classes needed for subsystems used with an opmode
     */
    public Arm(HydraOpMode opMode) {
        mOp = opMode;
        mControl = mOp.mOperatorGamepad;
        mLiftMotor = opMode.mHardwareMap.get(DcMotorEx.class, "liftMotor");
        mSlideMotor = opMode.mHardwareMap.get(DcMotorEx.class, "slideMotor");
        mWristServo = opMode.mHardwareMap.get(Servo.class, "wristServo");
        mExtendHomeSwitch = opMode.mHardwareMap.get(RevTouchSensor.class, "extendHomeSwitch");
       // mLiftHomeSwitch = opMode.mHardwareMap.get(RevTouchSensor.class, "liftHomeSwitch");
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
        mManualMode = false;
        mServoPosition = 0.0;
        mAction = ArmActions.Idle;
        mMoveState = ArmMoveStates.ExtendHome;
        mArmPosIdx = 0;
        mManualExtendInput = 0.0;
        mLastActiveAction = ArmActions.Idle;
        mWristServoManualIncrement = (mWristServoMaxPos - mWristServoMinPos) / (mWristServoTravelTime / mOp.mLoopTime);
        mManualWristInput = 0.0;
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
        // keep this around for later math
        mArmExtendInches = inches;
    }

    /**
     * Set the position of the wrist
     * @param pos: pos of the servo
     */
    public void SetWristPos(double pos) {
        // check the bounds of the input
        if (pos < mWristServoMinPos) {
            pos = mWristServoMinPos;
        } else if (pos > mWristServoMaxPos) {
            pos = mWristServoMaxPos;
        }
        mServoPosition = pos;
        mOp.mTelemetry.addData("servoPosition", mServoPosition);
    }

    /**
     * Call from main loop to handle user input in teleop
     * This is not used for auto opmodes
     */
    public void HandleUserInput() {
        mManualExtendInput = mControl.left_stick_y;
        mManualWristInput = mControl.right_stick_y;
        if (mManualExtendInput < Constants.joyStickDeadBand) {
            mManualExtendInput = 0;
        }
        if (mManualWristInput < Constants.joyStickDeadBand) {
            mManualWristInput = 0;
        }
        mManualMode = false;
        if (mControl.cross) {
            SetArmAction(ArmActions.RunHome);
        } else if (mControl.square) {
            SetArmAction(ArmActions.RunPickup);
        } else if (mControl.circle) {
            SetArmAction(ArmActions.RunScoreLow);
        } else if (mControl.triangle) {
            SetArmAction(ArmActions.RunScoreHigh);
        } else if (Math.abs(mManualExtendInput) > Constants.joyStickDeadBand ||
                Math.abs(mManualWristInput) > Constants.joyStickDeadBand) {
            if (LiftHome() && ExtendHome()) {
                SetArmAction(ArmActions.RunManual);
                mManualMode = true;
            }
        } else {
            SetArmAction(ArmActions.Idle);
        }
    }

    public void SetArmAction(ArmActions action) {
        if (action != mAction) {
            if (action == ArmActions.Idle) {
                mLastActiveAction = mAction;
            }
            mAction = action;
            switch (mAction) {
                case RunHome:
                    mArmPosIdx = ArmPositions.valueOf("Pos0Home").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunPickup:
                    if (mOp.mTargetElement == ElementTypes.Sample) {
                        mArmPosIdx = ArmPositions.valueOf("Pos2FloorPickup").ordinal();
                    } else {
                        mArmPosIdx = ArmPositions.valueOf("Pos3SpecimenPickup").ordinal();
                    }
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunScoreLow:
                    if (mOp.mTargetElement == ElementTypes.Sample) {
                        mArmPosIdx = ArmPositions.valueOf("Pos4SpecimenLowerChamber").ordinal();
                    } else {
                        mArmPosIdx = ArmPositions.valueOf("Pos6SampleLowerBasket").ordinal();
                    }
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunScoreHigh:
                    if (mOp.mTargetElement == ElementTypes.Sample) {
                        mArmPosIdx = ArmPositions.valueOf("Pos5SpecimenUpperChamber").ordinal();
                    } else {
                        mArmPosIdx = ArmPositions.valueOf("Pos7SampleUpperBasket").ordinal();
                    }
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case Idle:
                case RunManual:
                    break;
            }
        }
    }

    /**
     * Calls processes for each system in this subsystem
     * Handles automatic set angles based on other inputs
     */
    public void Process() {
        switch (mAction) {
            case RunHome:
                SetWristPos(Pos0Home_Wrist);
                if (!ExtendHome()) {
                    mArmExtendInches = Pos0Home_Extend;
                } else if (!LiftHome()) {
                    mLiftPositionDeg = Pos0Home_Lift;
                }
                break;
            case RunManual:
                // override the set angle and set based on the extension
                // hypotenuse is the arm
                // opposite is the parallel side of the vertical we are mounted to (extended to the floor)
                // use sine to get the set angle
                // -1 gets us the negative angle we're looking for
                SetLiftArmAngle(Math.asin(-1 * mArmPivotHeightInches / (mArmBaseLenInches + mArmExtendInches)));
                break;
            case Idle:
                // stop all mechanisms at their current positions
                if (!TuneMode()) {
                    if (LiftBusy()) {
                        mLiftPositionTicks = mLiftMotor.getCurrentPosition();
                        mLiftPositionDeg = mLiftZeroPosDeg + mLiftPositionTicks / mLiftTicksPerDegree;
                    }
                    if (ExtendBusy()) {
                        mArmExtendTicks = mSlideMotor.getCurrentPosition();
                        mArmExtendInches = mArmExtendTicks / mArmExtendTicksPerInch;
                    }
                }
                break;
            case RunPickup:
            case RunScoreHigh:
            case RunScoreLow:
                switch (mMoveState) {
                    case ExtendHome:
                        if (ExtendHome()) {
                            mMoveState = ArmMoveStates.LiftAngle;
                            mLiftPositionDeg = mLiftPositions[mArmPosIdx];
                            mServoPosition = mWristPositions[mArmPosIdx];
                        } else {
                            mArmExtendInches = 0;
                        }
                        break;
                    case LiftAngle:
                        if (!LiftBusy()) {
                            mMoveState = ArmMoveStates.ExtendToPos;
                            mArmExtendInches = mExtendPositions[mArmPosIdx];
                        }
                        break;
                    case ExtendToPos:
                        if (!ExtendBusy()) {
                            mMoveState = ArmMoveStates.Done;
                        }
                        break;
                    case Done:
                        mAction = ArmActions.Idle;
                        break;
                }
                break;
        }
        // Processes for each system
        ProcessArmAngle();
        ProcessArmExtension();
        ProcessWristPosition();
    }

    /**
     * Updates the arm angle if it has changed
     */
    private void ProcessArmAngle() {
        mPID.setPID(mLiftP, mLiftI, mLiftD);
        // Get current position for calculations
        int currentPos = mLiftMotor.getCurrentPosition();
        // Calculate the pid to get from current position to desired
        double pid = mPID.calculate(currentPos, mLiftPositionTicks);
        // Factor in gravity. Cos will scale appropriately based on the angle
        double extensionPct = (mArmExtendMaxInches - mArmExtendInches) / mArmExtendMaxInches;
        double f = mLiftFRetracted + extensionPct * (mLiftFExtended - mLiftFRetracted);
        double ff = Math.cos(Math.toRadians(mLiftPositionDeg)) * f;
        // Add pid and feed forward to get the final power
        double power = pid + ff;
        // Set power to the motor
        mLiftMotor.setPower(power);
        // Telemetry for debugging
        mOp.mTelemetry.addData("F", f);
        mOp.mTelemetry.addData("Lift Pos", currentPos);
        mOp.mTelemetry.addData("Lift Target", mLiftPositionTicks);
        mOp.mTelemetry.addData("Lift Power", power);
    }

    /**
     * Updates the extension if it has changed
     */
    private void ProcessArmExtension() {
        // Get the current slide position to compare
        int currentSlide = mSlideMotor.getCurrentPosition();
        if (mManualMode) {
            mSlideMotor.setPower(mManualExtendInput);
        } else {
            // If we're close then we don't need to do anything
            if (Math.abs(currentSlide - mArmExtendTicks) > 5) {
                mSlideMotor.setTargetPosition(mArmExtendTicks);
                mSlideMotor.setPower(mSlideMotorPower);
                mSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                mSlideMotor.setPower(0);
            }
        }
        // Telemetry for debugging
        mOp.mTelemetry.addData("Slide Pos", currentSlide);
        mOp.mTelemetry.addData("Slide Target", mArmExtendTicks);
    }

    /**
     * Sets the wrist position
     */
    private void ProcessWristPosition() {
        if (mManualMode) {
            SetWristPos(mServoPosition + mWristServoManualIncrement * mManualWristInput);
        }
        mWristServo.setPosition(mServoPosition);
    }

    /**
     * Returns whether the lift motor is busy
     * @return true when the lift motor is busy
     */
    public boolean LiftBusy() {
        // we can't use the busy from the motor because of gravity
        // we have to use the position instead
        return Math.abs(mLiftMotor.getCurrentPosition() - mLiftPositionTicks) < 15;
    }

    /**
     * Returns whether the extension motor is busy
     * @return true when running to position
     */
    public boolean ExtendBusy() {
        return mSlideMotor.isBusy();
    }

    /**
     * Returns whether the wrist servo is busy
     * @return true when the wrist servo has not reached position
     */
    public boolean WristBusy() {
        return mWristServo.getPosition() == mServoPosition;
    }

    /**
     * Returns whether we are auto-setting the arm and wrist angles
     * @return true when auto-set of arm and wrist angles is enabled
     */
    public boolean TuneMode() {
        return mEnableTune;
    }

    /**
     * Returns whether the arm lift is at home
     * @return true when the lift is at the home position
     */
    private boolean LiftHome() {
        if (mLiftHomeSwitch != null) {
            return mLiftHomeSwitch.isPressed();
        } else {
            return mLiftMotor.getCurrentPosition() < 15;
        }
    }
    /**
     * Returns whether the arm extension is home
     * @return true when the extension is at the home position
     */
    private boolean ExtendHome() {
        return mExtendHomeSwitch.isPressed();
    }
}
