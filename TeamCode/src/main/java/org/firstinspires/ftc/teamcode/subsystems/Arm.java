package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ArmMoveStates;
import org.firstinspires.ftc.teamcode.types.ArmPositions;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Config
public class Arm {
    private final HydraOpMode mOp;
    private final PIDController mPID;
    private final DcMotorEx mLiftMotor;
    private final DcMotorEx mSlideMotor;
    private final Servo mWristServo;
    private final RevTouchSensor mExtendHomeSwitch;
    private final RevTouchSensor mLiftHomeSwitch;
    private final com.qualcomm.robotcore.hardware.Gamepad mControl;
    // Lift arm PIDF controller gains
    private final double mLiftP = 0.004;
    private final double mLiftI = 0.002;
    private final double mLiftD = 0.0001;
    private final double mLiftFRetracted = 0.3;
    private final double mLiftFExtended = 0.7;
    // Lift arm motor ticks per degree
    // 1993.6 PPR at the motor
    // 2x1 gear
    private final double mLiftTicksPerDegree = 1993.6 / 180.0;
    // Lift arm desired position in ticks
    private int mLiftPositionTicks;
    // Lift arm motor zero position (home)
    public static double mLiftZeroPosDeg = -20.0;
    // Lift arm motor max lift position
    private final double mLiftMaxPosDeg = 99.0;
    // Slide motor desired position in ticks
    private int mArmExtendTicks;
    // 537.7 PPR at the motor
    // 4.72441 inches / revolution
    private final double mArmExtendTicksPerInch = 537.7 / 4.72441;
    // Max extension of the arm in inches
    // max physically possible 27.95 inches
    private final double mArmExtendMaxInches = 18.5;
    // Power level for slide motor
    private final double mSlideMotorPower = 1.0;
    // Whether or not we're currently utilizing manual extension to pick up samples
    private boolean mManualMode;
    // max position value for the wrist servo
    private final double mWristServoMaxPos = 0.78;
    // min position value for the wrist servo
    private final double mWristServoMinPos = 0.3;
    // min time in seconds for full traversing of wrist servo range
    private final double mWristServoTravelTime = 5;
    // how much to increment the wrist servo position by every time the loop runs
    private final double mWristServoManualIncrement;
    // manual controller input for the wrist servo
    private double mManualWristInput;
    // position to set for the wrist servo
    private double mServoPosition;
    public static boolean mEnableTune = false;
    private ArmActions mAction;
    private ArmActions mLastActiveAction;
    // predetermined lift and extend positions for all arm positions
    public static double Pos0Home_Lift = mLiftZeroPosDeg;
    public static double Pos0Home_Extend = 0.0;
    public static double Pos0Home_Wrist = 0.4;
    public static double Pos1ManualPickup_Lift = -8.0;
    public static double Pos1ManualPickup_LiftExtended = 2.0;
    public static double Pos1ManualPickup_Extend = 0.0;
    public static double Pos1ManualPickup_Wrist = 0.55;
    public static double Pos2FloorPickup_Lift = 0.0;
    public static double Pos2FloorPickup_Extend = 0.0;
    public static double Pos2FloorPickup_Wrist = 0.55;
    public static double Pos3SpecimenPickup_Lift = 11;
    public static double Pos3SpecimenPickup_Extend = 0.0;
    public static double Pos3SpecimenPickup_Wrist = 0.48;
    public static double Pos4SpecimenLowerChamber_Lift = 40.0;
    public static double Pos4SpecimenLowerChamber_Extend = 0.0;
    public static double Pos4SpecimenLowerChamber_Wrist = 0.7;
    public static double Pos5SpecimenUpperChamber_Lift = 55.0;
    public static double Pos5SpecimenUpperChamber_Extend = 13.0;
    public static double Pos5SpecimenUpperChamber_Wrist = 0.8;
    public static double Pos6SampleLowerBasket_Lift = 99.0;
    public static double Pos6SampleLowerBasket_Extend = 0.0;
    public static double Pos6SampleLowerBasket_Wrist = 0.3;
    public static double Pos7SampleUpperBasket_Lift = 99.0;
    public static double Pos7SampleUpperBasket_Extend = 18.5;
    public static double Pos7SampleUpperBasket_Wrist = 0.3;
    public static double Pos8Carry_Lift = 20.0;
    public static double Pos8Carry_Extend = 0.0;
    public static double Pos8Carry_Wrist = 0.45;
    public static double SpecimenLowDropAngle1 = 0.0;
    private ElapsedTime mHighSpecimenWristWait;
    // create arrays with the preset values for quick lookup
    public static double[] mLiftPositions = { Pos0Home_Lift, Pos1ManualPickup_Lift, Pos2FloorPickup_Lift, Pos3SpecimenPickup_Lift,
            Pos4SpecimenLowerChamber_Lift, Pos5SpecimenUpperChamber_Lift, Pos6SampleLowerBasket_Lift, Pos7SampleUpperBasket_Lift,
            Pos8Carry_Lift };
    public static double[] mExtendPositions = { Pos0Home_Extend, Pos1ManualPickup_Extend, Pos2FloorPickup_Extend, Pos3SpecimenPickup_Extend,
            Pos4SpecimenLowerChamber_Extend, Pos5SpecimenUpperChamber_Extend, Pos6SampleLowerBasket_Extend, Pos7SampleUpperBasket_Extend,
            Pos8Carry_Extend };
    public static double[] mWristPositions = { Pos0Home_Wrist, Pos1ManualPickup_Wrist, Pos2FloorPickup_Wrist, Pos3SpecimenPickup_Wrist,
            Pos4SpecimenLowerChamber_Wrist, Pos5SpecimenUpperChamber_Wrist, Pos6SampleLowerBasket_Wrist, Pos7SampleUpperBasket_Wrist,
            Pos8Carry_Wrist };
    // index into the position arrays for current movement
    private int mArmPosIdx;
    // current state of an arm movement
    private ArmMoveStates mMoveState;
    // user supplied extension power
    private double mManualExtendInput;
    // state for running to home at the beginning of the opmode
    private int mArmResetState;
    // timeout to use for resetting the arm
    ElapsedTime mArmResetTimer;
    // button debouncers
    private final int mDebounce = 3;
    Debouncer mDpadDown;
    Debouncer mDpadLeft;
    Debouncer mCross;
    Debouncer mSquare;
    Debouncer mCircle;
    Debouncer mTriangle;

    /**
     * Initializes the Arm object
     * Lift and extension
     * @param opMode contains various classes needed for subsystems used with an opmode
     */
    public Arm(HydraOpMode opMode) {
        mOp = opMode;
        mControl = mOp.mOperatorGamepad;
        mLiftMotor = mOp.mHardwareMap.get(DcMotorEx.class, "liftMotor");
        mSlideMotor = mOp.mHardwareMap.get(DcMotorEx.class, "slideMotor");
        mWristServo = mOp.mHardwareMap.get(Servo.class, "wristServo");
        mExtendHomeSwitch = mOp.mHardwareMap.get(RevTouchSensor.class, "extendHomeSwitch");
        mLiftHomeSwitch = mOp.mHardwareMap.get(RevTouchSensor.class, "liftHomeSwitch");
        mPID = new PIDController(mLiftP, mLiftI, mLiftD);
        mLiftPositionTicks = 0;
        mArmExtendTicks = 0;
        mManualMode = false;
        mServoPosition = Pos0Home_Wrist;
        mAction = ArmActions.Idle;
        mMoveState = ArmMoveStates.Done;
        mArmPosIdx = 0;
        mManualExtendInput = 0.0;
        mLastActiveAction = ArmActions.Idle;
        mWristServoManualIncrement = (mWristServoMaxPos - mWristServoMinPos) / (mWristServoTravelTime / (double)mOp.mLoopTime);
        mManualWristInput = 0.0;
        mArmResetState = 0;
        mArmResetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mHighSpecimenWristWait = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mDpadDown = new Debouncer(mDebounce);
        mDpadLeft = new Debouncer(9);
        mCross = new Debouncer(mDebounce);
        mSquare = new Debouncer(mDebounce);
        mTriangle = new Debouncer(mDebounce);
        mCircle = new Debouncer(mDebounce);
    }

    /**
     * Gets the robot to the home position
     * Call once before entering the main loop, blocking with timeouts
     */
    public boolean Startup(boolean handOff) {
        switch (mArmResetState) {
            case 0:
                mArmResetTimer.reset();
                if (handOff) {
                    // another opmode already ran, don't need to unfold
                    mArmResetState = 7;
                } else {
                    // since this is a fresh opmode, reset everything
                    mLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    mLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    mSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    mSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    // start by lifting the arm so we can get the intake out of the way
                    mLiftMotor.setTargetPosition((int)(mLiftMotor.getCurrentPosition() + 10 * mLiftTicksPerDegree));
                    mLiftMotor.setPower(0.3);
                    mLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mArmResetState = 1;
                }
                break;
            case 1:
                // wait for the lift to get to the position we set
                if (LiftBusy()) {
                    break;
                } else {
                    mArmResetState = 2;
                    // fall through
                }
            case 2:
                // set the wrist servo to the home position
                // there is nothing else to do since the servo position is always absolute
                SetWristPos(Pos0Home_Wrist);
                ProcessWristPosition();
                mArmResetState = 3;
                // fall through
            case 3:
                // run the slides to the home position
                if (ExtendHome(true)) {
                    mArmResetState = 4;
                    // fall through
                } else {
                    // go by 1/2 in at a time until we get there
                    mSlideMotor.setTargetPosition((int)(mSlideMotor.getCurrentPosition() - 0.5 * mArmExtendTicksPerInch));
                    mSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mSlideMotor.setPower(0.3);
                    break;
                }
            case 4:
                // this might need to be replaced with capturing the ticks and offsetting
                mSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // set to the home position now so we get all the state set correctly
                SetArmExtension(Pos0Home_Extend);
                ProcessArmExtension();
                mArmResetState = 5;
                // fall through
            case 5:
                if (LiftHome(true)) {
                    mArmResetState = 6;
                    // fall through
                } else {
                    mLiftMotor.setTargetPosition((int)(mLiftMotor.getCurrentPosition() - 5 * mLiftTicksPerDegree));
                    mLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mLiftMotor.setPower(0.3);
                    break;
                }
            case 6:
                // this might need to be replaced with capturing the ticks and offsetting
                mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // set to the home position now so we get all the state set correctly
                SetLiftArmAngle(Pos0Home_Lift);
                ProcessArmAngle();
                mArmResetState = 7;
                break;
            default:
                return true;
        }
        if (mArmResetTimer.milliseconds() >= 5000) {
            return true;
        }
        return false;
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
    }

    /**
     * Calculates the arm angle from ticks
     * @param ticks: ticks to calculate from
     * @return Angle of the arm in degrees
     */
    private double GetLiftAngleFromTicks(int ticks) {
        return mLiftZeroPosDeg + (double)ticks / mLiftTicksPerDegree;
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
    }

    /**
     * Calculates the extension from ticks
     * @param ticks: ticks to calculate from
     * @return Inches of arm extension based on the tick value
     */
    private double GetExtensionFromTicks(int ticks) {
        return (double)ticks / mArmExtendTicksPerInch;
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
        // capture the current joystick values
        mManualExtendInput = -mControl.left_stick_y;
        mManualWristInput = mControl.right_stick_y;
        // place a dead band around the center of the joysticks
        boolean joystickValid = false;
        if (Math.abs(mManualExtendInput) < Constants.joyStickDeadBand) {
            mManualExtendInput = 0;
        } else {
            joystickValid = true;
        }
        if (Math.abs(mManualWristInput) < Constants.joyStickDeadBand) {
            mManualWristInput = 0;
        } else {
            joystickValid = true;
        }
        // run buttons through debouncers
        mDpadDown.In(mControl.dpad_down);
        mDpadLeft.In(mControl.dpad_left);
        mCross.In(mControl.cross);
        // determine which action the user wants to perform
        if (mMoveState == ArmMoveStates.Done) {
            if (mCross.Out()) {
                mCross.Used();
                SetArmAction(ArmActions.RunCarry);
                mManualMode = false;
            } else if (mSquare.Out()) {
                mSquare.Used();
                SetArmAction(ArmActions.RunPickup);
                mManualMode = false;
            } else if (mCircle.Out()) {
                mCircle.Used();
                SetArmAction(ArmActions.RunScoreLow);
                mManualMode = false;
            } else if (mTriangle.Out()) {
                mTriangle.Used();
                SetArmAction(ArmActions.RunScoreHigh);
                mManualMode = false;
            } else if (mDpadLeft.Out()) {
                mDpadLeft.Used();
                SetArmAction(ArmActions.RunHome);
                mManualMode = false;
            } else if (mManualMode || joystickValid) {
                if (mLastActiveAction != ArmActions.RunScoreHigh && mLastActiveAction != ArmActions.RunScoreLow) {
                    SetArmAction(ArmActions.RunManual);
                    mManualMode = true;
                }
            } else {
                // keep manual mode state so we maintain position
                SetArmAction(ArmActions.Idle);
            }
        }
        mOp.mTelemetry.addData("Manual", mManualMode);
        mOp.mTelemetry.addData("Manual Extend", mManualExtendInput);
        mOp.mTelemetry.addData("Manual Wrist", mManualWristInput);
    }

    /**
     * Set the next arm action to perform
     * @param action: sets the action to perform
     */
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
                        mArmPosIdx = ArmPositions.valueOf("Pos6SampleLowerBasket").ordinal();
                    } else {
                        mArmPosIdx = ArmPositions.valueOf("Pos4SpecimenLowerChamber").ordinal();
                    }
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunScoreHigh:
                    if (mOp.mTargetElement == ElementTypes.Sample) {
                        mArmPosIdx = ArmPositions.valueOf("Pos7SampleUpperBasket").ordinal();
                    } else {
                        mArmPosIdx = ArmPositions.valueOf("Pos5SpecimenUpperChamber").ordinal();
                    }
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunCarry:
                    mArmPosIdx = ArmPositions.valueOf("Pos8Carry").ordinal();
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
    public boolean Process() {
        boolean requestRunIntake = false;
        switch (mAction) {
            case RunHome:
                SetWristPos(Pos0Home_Wrist);
                if (!ExtendHome(false)) {
                    SetArmExtension(Pos0Home_Extend);
                } else if (!LiftHome(false)) {
                    SetLiftArmAngle(Pos0Home_Lift);
                } else {
                    mMoveState = ArmMoveStates.Done;
                }
                break;
            case RunManual:
                // override the set angle and set based on the extension
                // hypotenuse is the arm
                // opposite is the parallel side of the vertical we are mounted to (extended to the floor)
                // use sine to get the set angle
                // -1 gets us the negative angle we're looking for
                //double e = mArmBaseLenInches + GetExtensionFromTicks(mSlideMotor.getCurrentPosition());
               // SetLiftArmAngle(Math.asin(-1 * mArmPivotHeightInches / (e)));
                // another option that should keep a static height off of the floor
                //SetLiftArmAngle(Math.asin(-1 * mArmPivotHeightInches / (e + e / mManualArmAutoAngleRatio)));
                //SetLiftArmAngle(Pos1ManualPickup_Lift);
                //SetWristPos(Pos1ManualPickup_Wrist);
                break;
            case Idle:
                // stop all mechanisms at their current positions
                if (!TuneMode()) {
                   // SetLiftArmAngle(GetLiftAngleFromTicks(mLiftMotor.getCurrentPosition()));
                   // SetArmExtension(GetExtensionFromTicks(mSlideMotor.getCurrentPosition()));
                }
                break;
            case RunPickup:
            case RunScoreHigh:
            case RunScoreLow:
            case RunCarry:
                switch (mMoveState) {
                    case ExtendHome:
                        if (ExtendHome(false)) {
                            SetWristPos(Pos0Home_Wrist);
                            mMoveState = ArmMoveStates.LiftAngle;
                            SetLiftArmAngle(mLiftPositions[mArmPosIdx]);
                        } else {
                            SetArmExtension(Pos0Home_Extend);
                        }
                        break;
                    case LiftAngle:
                        if (!LiftBusy()) {
                            mMoveState = ArmMoveStates.ExtendToPos;
                            SetArmExtension(mExtendPositions[mArmPosIdx]);
                        }
                        break;
                    case ExtendToPos:
                        if (!ExtendBusy()) {
                            if (mOp.mTargetElement == ElementTypes.Specimen && (mAction == ArmActions.RunScoreHigh || mAction == ArmActions.RunScoreLow)) {
                                SetWristPos(0.67);
                                mMoveState = ArmMoveStates.SpecimenWait1;
                            } else {
                                SetWristPos(mWristPositions[mArmPosIdx]);
                                mMoveState = ArmMoveStates.Done;
                            }
                        }
                        break;
                    case SpecimenWait1:
                        if (mDpadDown.Out()) {
                            mDpadDown.Used();
                            if (mAction == ArmActions.RunScoreHigh) {
                                SetWristPos(mWristPositions[mArmPosIdx]);
                                mMoveState = ArmMoveStates.SpecimenWait2;
                                mHighSpecimenWristWait.reset();
                            } else {
                                SetLiftArmAngle(SpecimenLowDropAngle1);
                                mMoveState = ArmMoveStates.SpecimenDropLow;
                            }
                        }
                        break;
                    case SpecimenWait2:
                        if (mDpadDown.Out()) {
                            mDpadDown.Used();
                            SetArmExtension(0.0);
                            requestRunIntake = true;
                            mMoveState = ArmMoveStates.SpecimenDropHigh;
                        }
                        break;
                    case SpecimenDropHigh:
                        if (!ExtendBusy()) {
                            mMoveState = ArmMoveStates.Done;
                        }
                        break;
                    case SpecimenDropLow:
                        if (!LiftBusy()) {
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
        mOp.mTelemetry.addData("Lift Home (sw)", LiftHome(true));
        mOp.mTelemetry.addData("Lift Home (pos)", LiftHome(false));
        mOp.mTelemetry.addData("Extend Home (sw)", ExtendHome(true));
        mOp.mTelemetry.addData("Extend Home (pos)", ExtendHome(false));
        mOp.mTelemetry.addData("Action", mAction);
        mOp.mTelemetry.addData("State", mMoveState);
        mOp.mTelemetry.addData("Index", mArmPosIdx);
        mOp.mTelemetry.addData("Lift Current", mLiftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        mOp.mTelemetry.addData("Extend Current", mSlideMotor.getCurrent(CurrentUnit.MILLIAMPS));
        return requestRunIntake;
    }

    /**
     * Updates the arm angle if it has changed
     */
    private void ProcessArmAngle() {
        // Get current position for calculations
        int currentPos = mLiftMotor.getCurrentPosition();
        // Get the current angle of the arm
        double currentPosDeg = GetLiftAngleFromTicks(currentPos);
        double power = 0;
        if (!mLiftHomeSwitch.isPressed() && (mLiftPositionTicks == 0)) {
            mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            power = 0;
        } else {
            // Get percentage of arm extension
            double extensionPct = GetExtensionFromTicks(mSlideMotor.getCurrentPosition()) / mArmExtendMaxInches;
            if (mManualMode) {
                double angle = Pos1ManualPickup_Lift + (Pos1ManualPickup_LiftExtended - Pos1ManualPickup_Lift) * extensionPct;
                SetLiftArmAngle(angle);
            }
            mPID.setPID(mLiftP, mLiftI, mLiftD);
            // Calculate the pid to get from current position to desired
            double pid = mPID.calculate(currentPos, mLiftPositionTicks);
            // Factor in gravity
            // The force from gravity is higher when the arm is extended
            // Calculate f linearly based on how far the arm is extended
            double f = mLiftFRetracted + extensionPct * (mLiftFExtended - mLiftFRetracted);
            // Cos will scale the force of gravity based on the current arm angle
            double ff = Math.cos(Math.toRadians(GetLiftAngleFromTicks(mLiftPositionTicks))) * f;
            // Add pid and feed forward to get the final power
            power = pid + ff;
        }
        mLiftMotor.setPower(power);
        mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Telemetry for debugging
        //mOp.mTelemetry.addData("F", f);
        mOp.mTelemetry.addData("Lift Pos", currentPos);
        mOp.mTelemetry.addData("Lift Target", mLiftPositionTicks);
        mOp.mTelemetry.addData("Lift Power", power);
        mOp.mTelemetry.addData("Lift Pos (deg)", currentPosDeg);
    }

    /**
     * Updates the extension if it has changed
     */
    private void ProcessArmExtension() {
        int current = mSlideMotor.getCurrentPosition();
        if (mManualMode) {
            double power = 0;
            if (mManualExtendInput > 0) {
                if (GetExtensionFromTicks(current) < mArmExtendMaxInches) {
                    power = mManualExtendInput;
                }
            } else {
                if (GetExtensionFromTicks(current) > 0) {
                    power = mManualExtendInput;
                }
            }
            mSlideMotor.setPower(power);
            mSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        } else if (mExtendHomeSwitch.isPressed() && (mArmExtendTicks == 0)) {
            mSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSlideMotor.setPower(0);
            mSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            mSlideMotor.setTargetPosition(mArmExtendTicks);
            mSlideMotor.setPower(mSlideMotorPower);
            mSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        // Telemetry for debugging
        mOp.mTelemetry.addData("Slide Pos", current);
        mOp.mTelemetry.addData("Slide Target", mArmExtendTicks);
    }

    /**
     * Sets the wrist position
     */
    private void ProcessWristPosition() {
        if (mManualMode) {
            SetWristPos(Pos1ManualPickup_Wrist);
            //SetWristPos(mServoPosition + mWristServoManualIncrement * mManualWristInput);
        }
        mWristServo.setPosition(mServoPosition);
    }

    /**
     * Returns whether the lift motor is busy
     * @return true when running to position
     */
    public boolean LiftBusy() {
        // we can't use the busy from the motor because we're running to position
        // we have to use the position instead
        return Math.abs(mLiftMotor.getCurrentPosition() - mLiftPositionTicks) > 40;
    }

    /**
     * Returns whether the extension motor is busy
     * @return true when running to position
     */
    public boolean ExtendBusy() {
        // we can't use the busy from the motor because we're running to position
        // we have to use the position instead
        return Math.abs(mSlideMotor.getCurrentPosition() - mArmExtendTicks) > 40;
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
    private boolean LiftHome(boolean useSwitch) {
        if (useSwitch) {
            return !mLiftHomeSwitch.isPressed();
        } else {
            return Math.abs(GetLiftAngleFromTicks(mLiftMotor.getCurrentPosition()) - Pos0Home_Lift) <= 5;
        }
    }
    /**
     * Returns whether the arm extension is home
     * @return true when the extension is at the home position
     */
    private boolean ExtendHome(boolean useSwitch) {
        if (useSwitch) {
            return mExtendHomeSwitch.isPressed();
        } else {
            return GetExtensionFromTicks(mSlideMotor.getCurrentPosition()) <= 2.0;
        }
    }

    /*
     * ROAD RUNNER API
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetAction(ArmActions action) {
        return new RunAction(action);
    }

    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private final ArmActions mAction;
        // run has been called once
        private boolean started = false;

        // construct on the supplied action
        public RunAction(ArmActions action) {
            mAction = action;
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                SetArmAction(mAction);
                started = true;
            }
            Process();
            return mMoveState != ArmMoveStates.Done;
        }
    }
}
