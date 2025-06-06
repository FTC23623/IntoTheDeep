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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.datalogger.ArmDatalogger;
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
    private final PIDController mLiftPID;
    private final DcMotorEx mLiftMotor;
    private final DcMotorEx mSlideMotor;
    private final Servo mWristServo;
    private final RevTouchSensor mExtendHomeSwitch;
    private final RevTouchSensor mLiftHomeSwitch;
    private final com.qualcomm.robotcore.hardware.Gamepad mControl;
    private final ArmDatalogger mArmDatalogger;
    private final ElapsedTime mHighSpecimenWristWait;
    // Lift arm PIDF controller gains
    private final double mLiftP = 0.004;
    private final double mLiftI = 0.00352;
    private final double mLiftD = 0.0002;
    private final double mLiftFRetracted = 0.35;
    private final double mLiftFExtended = 0.6;
    // Lift arm motor ticks per degree
    // 1993.6 PPR at the motor
    // 2x1 gear
    private final double mLiftTicksPerDegree = 1993.6 / 180.0;
    private final double mLiftIntegralRangeDeg = 20.0;
    private final int mLiftIntegralZeroRange = 10 * (int)mLiftTicksPerDegree;
    // Lift arm desired position in ticks
    private int mLiftPositionTicks;
    // Lift arm motor zero position (home)
    private final double mLiftZeroPosDeg = -20.0;
    // Lift arm motor max lift position
    private final double mLiftMaxPosDeg = 115.0;
    // Slide motor desired position in ticks
    private int mArmExtendTicks;
    // 384.5 PPR at the motor
    // 4.72441 inches / revolution
    private final double mArmExtendTicksPerInch = 384.5 / 4.72441;
    // Max extension of the arm in inches
    // max physically possible 27.95 inches
    private final double mArmExtendMaxInches = 18.5;
    private final double mExtendForwardMax = 12.5;
    // Whether or not we're currently utilizing manual extension to pick up samples
    private boolean mManualMode;
    // max position value for the wrist servo
    private final double mWristServoMaxPos = 0.78;
    // min position value for the wrist servo
    private final double mWristServoMinPos = 0.3;
    // manual controller input for the wrist servo
    private double mManualWristInput;
    // position to set for the wrist servo
    private double mServoPosition;
    public static boolean mEnableTune = false;
    private ArmActions mAction;
    private ArmActions mLastActiveAction;
    // predetermined lift and extend positions for all arm positions
    private final double Pos0Home_Lift = mLiftZeroPosDeg;
    private final double Pos0Home_Extend = 0.0;
    private final double Pos0Home_Wrist = 0.4;
    private final double Pos1ManualPickup_Lift = -8.0;
    private final double Pos1ManualPickup_LiftExtended = 0.0;
    private final double Pos1ManualPickup_Extend = 0.0;
    private final double Pos1ManualPickup_Wrist = mWristServoMinPos + (mWristServoMaxPos - mWristServoMinPos) / 2;
    private final double Pos2FloorPickup_Lift = 0.0;
    private final double Pos2FloorPickup_Extend = 0.0;
    private final double Pos2FloorPickup_Wrist = 0.55;
    private final double Pos3SpecimenPickup_Lift = 11;
    private final double Pos3SpecimenPickup_Extend = 0.0;
    private final double Pos3SpecimenPickup_Wrist = 0.48;
    private final double Pos4SpecimenLowerChamber_Lift = 40.0;
    private final double Pos4SpecimenLowerChamber_Extend = 0.0;
    private final double Pos4SpecimenLowerChamber_Wrist = 0.7;
    private final double Pos5SpecimenUpperChamber_Lift = 55.0;
    private final double Pos5SpecimenUpperChamber_Extend = 13.0;
    private final double Pos5SpecimenUpperChamber_Wrist = 0.8;
    private final double Pos6SampleLowerBasket_Lift = 99.0;
    private final double Pos6SampleLowerBasket_Extend = 0.0;
    private final double Pos6SampleLowerBasket_Wrist = 0.3;
    private final double Pos7SampleUpperBasket_Lift = 99.0;
    private final double Pos7SampleUpperBasket_Extend = 18.5;
    private final double Pos7SampleUpperBasket_Wrist = 0.3;
    private final double Pos8Carry_Lift = 20.0;
    private final double Pos8Carry_Extend = 0.0;
    private final double Pos8Carry_Wrist = 0.45;
    private final double Pos9Ascent1_Lift = 42.0;
    private final double Pos9Ascent1_Extend = 12.0;
    private final double Pos9Ascent1_Wrist = 0.45;
    private final double Pos10AutoSamplePickup_Lift = -5;
    private final double Pos10AutoSamplePickup_Extend = 11;
    private final double Pos10AutoSamplePickup_Wrist = 0.70;
    private final double Pos11Turtle_Lift = 230.0;
    private final double Pos11Turtle_Extend = 0.0;
    private final double Pos11Turtle_Wrist = 0.8;
    private final double Pos12AutoSamplePush_Lift = -12.0;
    private final double Pos12AutoSamplePush_Extend = 13.5;
    private final double Pos12AutoSamplePush_Wrist = 0.43;
    private final double Pos13AutoSpecSafe_Lift = 80.0;
    private final double Pos13AutoSpecSafe_Extend = 0.0;
    private final double Pos13AutoSpecSafe_Wrist = 0.45;
    private final double Pos14Ascent2_Lift = 115.0;
    private final double Pos14Ascent2_Extend = 3.0;
    private final double Pos14Ascent2_Wrist = 0.46;
    private final double Pos15Ascent2Close_Lift = 0.0;
    private final double Pos15Ascent2Close_Extend = 3.0;
    private final double Pos15Ascent2Close_Wrist = 0.46;
    private final double SpecimenLowDropAngle1 = 0.0;
    private final double ManualWristHalfRange = mWristServoMaxPos - Pos1ManualPickup_Wrist;
    // create arrays with the preset values for quick lookup
    private final double[] mLiftPositions = { Pos0Home_Lift, Pos1ManualPickup_Lift, Pos2FloorPickup_Lift, Pos3SpecimenPickup_Lift,
            Pos4SpecimenLowerChamber_Lift, Pos5SpecimenUpperChamber_Lift, Pos6SampleLowerBasket_Lift, Pos7SampleUpperBasket_Lift,
            Pos8Carry_Lift, Pos9Ascent1_Lift, Pos10AutoSamplePickup_Lift, Pos11Turtle_Lift, Pos12AutoSamplePush_Lift, Pos13AutoSpecSafe_Lift,
            Pos14Ascent2_Lift, Pos15Ascent2Close_Lift };
    private final double[] mExtendPositions = { Pos0Home_Extend, Pos1ManualPickup_Extend, Pos2FloorPickup_Extend, Pos3SpecimenPickup_Extend,
            Pos4SpecimenLowerChamber_Extend, Pos5SpecimenUpperChamber_Extend, Pos6SampleLowerBasket_Extend, Pos7SampleUpperBasket_Extend,
            Pos8Carry_Extend, Pos9Ascent1_Extend, Pos10AutoSamplePickup_Extend, Pos11Turtle_Extend, Pos12AutoSamplePush_Extend, Pos13AutoSpecSafe_Extend,
            Pos14Ascent2_Extend, Pos15Ascent2Close_Extend };
    private final double[] mWristPositions = { Pos0Home_Wrist, Pos1ManualPickup_Wrist, Pos2FloorPickup_Wrist, Pos3SpecimenPickup_Wrist,
            Pos4SpecimenLowerChamber_Wrist, Pos5SpecimenUpperChamber_Wrist, Pos6SampleLowerBasket_Wrist, Pos7SampleUpperBasket_Wrist,
            Pos8Carry_Wrist, Pos9Ascent1_Wrist, Pos10AutoSamplePickup_Wrist, Pos11Turtle_Wrist, Pos12AutoSamplePush_Wrist, Pos13AutoSpecSafe_Wrist,
            Pos14Ascent2_Wrist, Pos15Ascent2Close_Wrist };
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
    Debouncer mDpadDown;
    Debouncer mDpadLeft;
    Debouncer mDpadUp;
    Debouncer mDpadRight;
    Debouncer mCross;
    Debouncer mSquare;
    Debouncer mCircle;
    Debouncer mTriangle;
    Debouncer mSquareToCancel;
    Debouncer mLeftBumper;
    private boolean mTriedFullReset;
    private double mAutoWristOverride;

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
        mLiftPID = new PIDController(mLiftP, mLiftI, mLiftD);
        mArmDatalogger = new ArmDatalogger("ArmLog");
        mLiftPositionTicks = 0;
        mArmExtendTicks = 0;
        mManualMode = false;
        mServoPosition = Pos0Home_Wrist;
        mAction = ArmActions.Idle;
        mMoveState = ArmMoveStates.Done;
        mArmPosIdx = 0;
        mManualExtendInput = 0.0;
        mLastActiveAction = ArmActions.Idle;
        mManualWristInput = 0.0;
        mArmResetState = 0;
        mArmResetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mHighSpecimenWristWait = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mDpadDown = new Debouncer(Constants.debounce);
        mDpadLeft = new Debouncer(Constants.debounceLong);
        mDpadUp = new Debouncer(Constants.debounce);
        mDpadRight = new Debouncer(Constants.debounceLong * 4);
        mCross = new Debouncer(Constants.debounce);
        mSquare = new Debouncer(Constants.debounce);
        mTriangle = new Debouncer(Constants.debounce);
        mCircle = new Debouncer(Constants.debounce);
        mSquareToCancel = new Debouncer(Constants.debounceLong);
        mLeftBumper = new Debouncer(Constants.debounce);
        mTriedFullReset = false;
        mAutoWristOverride = -1;
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
                    mLiftMotor.setTargetPosition((int)(20 * mLiftTicksPerDegree));
                    mLiftPositionTicks = (int)(20 * mLiftTicksPerDegree);
                    mLiftMotor.setPower(0.8);
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
                    mSlideMotor.setTargetPosition((int)(mSlideMotor.getCurrentPosition() - 2 * mArmExtendTicksPerInch));
                    mSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mSlideMotor.setPower(0.8);
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
                    mLiftMotor.setTargetPosition((int)(mLiftMotor.getCurrentPosition() - 10 * mLiftTicksPerDegree));
                    mLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mLiftMotor.setPower(0.8);
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
        return mArmResetTimer.milliseconds() >= 5000;
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

    public void SetArmAngleOffset(double degreesOffset) {
        SetLiftArmAngle(GetLiftAngleFromTicks(mLiftMotor.getCurrentPosition()) + degreesOffset);
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

    public void SetWristOffset(double posOffset) {
        SetWristPos(mServoPosition + posOffset);
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
        mDpadUp.In(mControl.dpad_up);
        mDpadRight.In(mControl.dpad_right);
        mCross.In(mControl.cross);
        mSquare.In(mControl.square);
        mCircle.In(mControl.circle);
        mTriangle.In(mControl.triangle);
        mSquareToCancel.In(mControl.square);
        mLeftBumper.In(mControl.left_bumper);
        // determine which action the user wants to perform
        if (mMoveState == ArmMoveStates.Done || mSquareToCancel.Out() || mLeftBumper.Out()) {
            if (mCross.Out()) {
                mCross.Used();
                SetArmAction(ArmActions.RunCarry);
            } else if (mSquare.Out()) {
                if (mMoveState != ArmMoveStates.Done) {
                    // move aborted. alert the operator
                    mControl.rumbleBlips(5);
                    mAction = ArmActions.Idle;
                    mLastActiveAction = ArmActions.Idle;
                }
                mSquare.Used();
                mSquareToCancel.Used();
                SetArmAction(ArmActions.RunPickup);
            } else if (mCircle.Out()) {
                mCircle.Used();
                SetArmAction(ArmActions.RunScoreLow);
            } else if (mTriangle.Out()) {
                mTriangle.Used();
                // if we're running specimens, the other arm consumes this action
                if (mOp.mTargetElement == ElementTypes.Sample) {
                    SetArmAction(ArmActions.RunScoreHigh);
                } else {
                    SetArmAction(ArmActions.RunPickup);
                }
            } else if (mDpadLeft.Out()) {
                mDpadLeft.Used();
                SetArmAction(ArmActions.RunHome);
            } else if (mDpadUp.Out()) {
                mDpadUp.Used();
                SetArmAction(ArmActions.RunAscent2);
            } else if (mDpadRight.Out()) {
                mDpadRight.Used();
                SetArmAction(ArmActions.RunTurtle);
            } else if (mLeftBumper.Out()) {
                mLeftBumper.Used();
                mAction = ArmActions.Idle;
                mLastActiveAction = ArmActions.Idle;
                SetArmAction(ArmActions.RunAscent2Close);
            } else if (mManualMode || joystickValid) {
                if (mLastActiveAction != ArmActions.RunScoreHigh && mLastActiveAction != ArmActions.RunScoreLow && mLastActiveAction != ArmActions.RunAscent2) {
                    SetArmAction(ArmActions.RunManual);
                }
            } else {
                // keep manual mode state so we maintain position
                SetArmAction(ArmActions.Idle);
            }
        }
        mOp.mTelemetry.addData("Manual", mManualMode);
    }

    /**
     * Set the next arm action to perform
     * @param action: sets the action to perform
     */
    public void SetArmAction(ArmActions action) {
        if (action != mAction) {
            if (action == ArmActions.Idle) {
                // when we switch to idle, keep track of the last thing we did
                // this stops us from repeating a previous action when we should already be in that position
                mLastActiveAction = mAction;
            } else {
                // this is in the else because we want to keep manual mode's value when we're idle
                // we're idle a lot when we're in manual mode
                mManualMode = action == ArmActions.RunManual;
            }
            // change the current action to the new one
            mAction = action;
            // set up the position indices for the new action to use in the state machine
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
                case RunAscent1:
                    mArmPosIdx = ArmPositions.valueOf("Pos9Ascent1").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunAutoSamplePickup:
                    mArmPosIdx = ArmPositions.valueOf("Pos10AutoSamplePickup").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunTurtle:
                    mArmPosIdx = ArmPositions.valueOf("Pos11Turtle").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunAutoSamplePush:
                    mArmPosIdx = ArmPositions.valueOf("Pos12AutoSamplePush").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunAutoSpecSafe:
                    mArmPosIdx = ArmPositions.valueOf("Pos13AutoSpecSafe").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunAscent2:
                    mArmPosIdx = ArmPositions.valueOf("Pos14Ascent2").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunAscent2Close:
                    mArmPosIdx = ArmPositions.valueOf("Pos15Ascent2Close").ordinal();
                    if (mLastActiveAction != mAction) {
                        mMoveState = ArmMoveStates.ExtendHome;
                    }
                    break;
                case RunManual:
                    mLastActiveAction = ArmActions.RunManual;
                    break;
                case Idle:
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
        boolean runProcesses = true;
        switch (mAction) {
            case RunHome:
                // first try to run home quickly by going to position
                // if that doesn't work, then run the arm startup process
                SetWristPos(Pos0Home_Wrist);
                if (!ExtendHome(false)) {
                    SetArmExtension(Pos0Home_Extend);
                } else if (!LiftHome(false)) {
                    SetLiftArmAngle(Pos0Home_Lift);
                } else if (!Startup(false)) {
                    // let the startup process run
                    runProcesses = false;
                } else if (!mTriedFullReset && (!ExtendHome(true) || !LiftHome(true))) {
                    // one of the mechanisms could not reach true home position
                    // reset arm reset state to 0 and start full reset
                    mArmResetState = 0;
                    Startup(false);
                    runProcesses = false;
                    mTriedFullReset = true;
                } else {
                    mMoveState = ArmMoveStates.Done;
                    mTriedFullReset = false;
                }
                break;
            case RunManual:
            case Idle:
                break;
            case RunPickup:
            case RunScoreHigh:
            case RunScoreLow:
            case RunCarry:
            case RunAscent1:
            case RunAutoSamplePickup:
            case RunTurtle:
            case RunAutoSamplePush:
            case RunAutoSpecSafe:
            case RunAscent2:
            case RunAscent2Close:
                switch (mMoveState) {
                    case ExtendHome:
                        if (mAction == ArmActions.RunAutoSamplePush || mAction == ArmActions.RunAscent2 || mAction == ArmActions.RunAscent2Close) {
                            SetLiftArmAngle(mLiftPositions[mArmPosIdx]);
                            if (mAction == ArmActions.RunAscent2) {
                                SetArmExtension(mExtendPositions[mArmPosIdx]);
                            }
                            mMoveState = ArmMoveStates.LiftAngle;
                            // fall through
                        } else {
                            if (ExtendHome(false)) {
                                SetWristPos(Pos0Home_Wrist);
                                mMoveState = ArmMoveStates.LiftAngle;
                                if (mOp.mTargetElement == ElementTypes.Specimen && mAction == ArmActions.RunScoreHigh) {
                                    SetLiftArmAngle(mLiftPositions[mArmPosIdx] - 5.0);
                                } else {
                                    SetLiftArmAngle(mLiftPositions[mArmPosIdx]);
                                }
                                // fall through
                            } else {
                                SetArmExtension(Pos0Home_Extend);
                                break;
                            }
                        }
                    case LiftAngle:
                        if (!LiftBusy()) {
                            mMoveState = ArmMoveStates.ExtendToPos;
                            SetArmExtension(mExtendPositions[mArmPosIdx]);
                            // fall through
                        } else {
                            break;
                        }
                    case ExtendToPos:
                        if (!ExtendBusy()) {
                            if (mOp.mTargetElement == ElementTypes.Specimen && (mAction == ArmActions.RunScoreHigh || mAction == ArmActions.RunScoreLow)) {
                                SetWristPos(0.71);
                                mMoveState = ArmMoveStates.SpecimenWait1;
                            } else {
                                if (mAutoWristOverride < 0) {
                                    SetWristPos(mWristPositions[mArmPosIdx]);
                                } else {
                                    SetWristPos(mAutoWristOverride);
                                    mAutoWristOverride = -1;
                                }
                                mMoveState = ArmMoveStates.Done;
                            }
                            // fall through
                        } else {
                            break;
                        }
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
                            // fall through
                        } else {
                            break;
                        }
                    case SpecimenWait2:
                        if (mDpadDown.Out()) {
                            mDpadDown.Used();
                            SetArmExtension(0.0);
                            requestRunIntake = true;
                            mMoveState = ArmMoveStates.SpecimenDropHigh;
                            // fall through
                        } else {
                            break;
                        }
                    case SpecimenDropHigh:
                        if (!ExtendBusy()) {
                            mMoveState = ArmMoveStates.Done;
                            // fall through
                        } else {
                            break;
                        }
                    case SpecimenDropLow:
                        if (!LiftBusy()) {
                            mMoveState = ArmMoveStates.Done;
                            // fall through
                        } else {
                            break;
                        }
                    case Done:
                        mAction = ArmActions.Idle;
                        break;
                }
                break;
        }
        if (runProcesses) {
            ProcessArmAngle();
            ProcessArmExtension();
            ProcessWristPosition();
        }
        mOp.mTelemetry.addData("Lift Home (sw)", LiftHome(true));
        mOp.mTelemetry.addData("Lift Home (pos)", LiftHome(false));
        mOp.mTelemetry.addData("Extend Home (sw)", ExtendHome(true));
        mOp.mTelemetry.addData("Extend Home (pos)", ExtendHome(false));
        mOp.mTelemetry.addData("Action", mAction);
        mOp.mTelemetry.addData("State", mMoveState);
        mOp.mTelemetry.addData("Index", mArmPosIdx);
        mArmDatalogger.action.set(mAction.toString());
        mArmDatalogger.state.set(mMoveState.toString());
        mArmDatalogger.battVoltage.set(mOp.mHardwareMap.get(VoltageSensor.class, "Control Hub").getVoltage());
        mArmDatalogger.writeLine();
        return requestRunIntake;
    }

    /**
     * Updates the arm angle if it has changed
     */
    private void ProcessArmAngle() {
        // Get current position for calculations
        int currentLiftPos = mLiftMotor.getCurrentPosition();
        // Get the current angle of the arm
        double currentPosDeg = GetLiftAngleFromTicks(currentLiftPos);
        double power;
        // trying to run home and active low switch is "pressed"
        // reset encoders and brake the motor
        if (LiftHome(true) && (mLiftPositionTicks == 0)) {
            mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mLiftPID.reset();
            power = 0;
        } else {
            // get the current arm extension in inches for a couple of calculations
            double currentExtensionInches = GetExtensionFromTicks(mSlideMotor.getCurrentPosition());
            if (mManualMode) {
                // automatically set the angle based on the extension
                double manualExtPct = Math.min(1.0, currentExtensionInches / mExtendForwardMax);
                double angle = Pos1ManualPickup_Lift + (Pos1ManualPickup_LiftExtended - Pos1ManualPickup_Lift) * manualExtPct;
                SetLiftArmAngle(angle);
            }
            mLiftPID.setPID(mLiftP, mLiftI, mLiftD);
            if (Math.abs(currentLiftPos - mLiftPositionTicks) > mLiftIntegralZeroRange) {
                mLiftPID.setIntegrationBounds(0, 0);
            } else {
                mLiftPID.setIntegrationBounds(-1 * mLiftIntegralRangeDeg * mLiftTicksPerDegree, mLiftIntegralRangeDeg * mLiftTicksPerDegree);
            }
            // Calculate the pid to get from current position to desired
            double pid = mLiftPID.calculate(currentLiftPos, mLiftPositionTicks);
            // Factor in gravity
            // The force from gravity is higher when the arm is extended
            // Get percentage of arm extension
            double extensionPct = Math.min(1.0, currentExtensionInches / mArmExtendMaxInches);
            // Calculate f linearly based on how far the arm is extended
            double f = mLiftFRetracted + extensionPct * (mLiftFExtended - mLiftFRetracted);
            // Cos will scale the force of gravity based on the current arm angle
            double ff = Math.cos(Math.toRadians(GetLiftAngleFromTicks(currentLiftPos))) * f;
            // Add pid and feed forward to get the final power
            power = pid + ff;
            mOp.mTelemetry.addData("Lift F", f);
            mOp.mTelemetry.addData("Lift FF", ff);
            mOp.mTelemetry.addData("Lift PID", pid);
        }
        mLiftMotor.setPower(power);
        mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Telemetry for debugging
        mOp.mTelemetry.addData("Lift Pos", currentLiftPos);
        mOp.mTelemetry.addData("Lift Target", mLiftPositionTicks);
        mOp.mTelemetry.addData("Lift Power", power);
        mOp.mTelemetry.addData("Lift Pos (deg)", currentPosDeg);
        double motorCurrent = mLiftMotor.getCurrent(CurrentUnit.MILLIAMPS);
        mOp.mTelemetry.addData("Lift Current", motorCurrent);
        mArmDatalogger.liftCurrent.set(motorCurrent);
        mArmDatalogger.liftPosition.set(currentLiftPos);
        mArmDatalogger.liftTarget.set(mLiftPositionTicks);
        mArmDatalogger.liftPower.set(power);
    }

    /**
     * Updates the extension if it has changed
     */
    private void ProcessArmExtension() {
        // Get current position for calculations
        int current = mSlideMotor.getCurrentPosition();
        if (mManualMode) {
            double power = 0;
            // cap the extension at min and max based on direction and mode
            if (mManualExtendInput > 0) {
                if (GetExtensionFromTicks(current) < mExtendForwardMax) {
                    power = mManualExtendInput;
                }
            } else {
                if (ExtendHome(true)) {
                    // reset the motor encoder and don't drive through home
                    mSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    power = 0;
                } else if (GetExtensionFromTicks(current) > 0) {
                    power = mManualExtendInput;
                }
            }
            mSlideMotor.setPower(power);
            mSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        } else if (ExtendHome(true) && (mArmExtendTicks == 0)) {
            // reset the motor encoder and don't drive through home
            mSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSlideMotor.setPower(0);
            mSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            mSlideMotor.setTargetPosition(mArmExtendTicks);
            mSlideMotor.setPower(1.0);
            mSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        // Telemetry for debugging
        //mOp.mTelemetry.addData("Extend Power", power);
        mOp.mTelemetry.addData("Extend Pos", current);
        mOp.mTelemetry.addData("Extend Target", mArmExtendTicks);
        double motorCurrent = mSlideMotor.getCurrent(CurrentUnit.MILLIAMPS);
        mOp.mTelemetry.addData("Extend Current", motorCurrent);
        mArmDatalogger.extendPosition.set(current);
        mArmDatalogger.extendTarget.set(mArmExtendTicks);
        //mArmDatalogger.extendPower.set(power);
        mArmDatalogger.extendCurrent.set(motorCurrent);
    }

    /**
     * Sets the wrist position
     */
    private void ProcessWristPosition() {
        if (mManualMode) {
            SetWristPos(Pos1ManualPickup_Wrist + ManualWristHalfRange * mManualWristInput);
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
     * Currently unused
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

    public Action GetActionWristOverride(ArmActions action, double wristOverride) {
        return new RunAction(action, wristOverride);
    }

    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private final ArmActions mRRAction;
        // run has been called once
        private boolean started = false;
        private double mWristOverride;

        // construct on the supplied action
        public RunAction(ArmActions action) {
            mRRAction = action;
            mWristOverride = -1;
        }

        public RunAction(ArmActions action, double wristOverride) {
            mRRAction = action;
            mWristOverride = wristOverride;
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                switch (mRRAction) {
                    case RunScoreHighOverBar:
                        SetArmAction(ArmActions.RunScoreHigh);
                        break;
                    case RunScoreHighDropWrist:
                    case RunScoreHighScore:
                        mDpadDown.Force();
                        break;
                    default:
                        SetArmAction(mRRAction);
                        break;
                }
                mAutoWristOverride = mWristOverride;
                started = true;
            }
            switch (mRRAction) {
                case RunScoreHighOverBar:
                    return mMoveState != ArmMoveStates.SpecimenWait1;
                case RunScoreHighDropWrist:
                    return mMoveState != ArmMoveStates.SpecimenWait2;
                default:
                    return mMoveState != ArmMoveStates.Done;
            }
        }
    }

    /**
     * Get a RR action that can adjust arm angle and wrist offset for pulling away from the basket
     * @param armOffsetDegrees: degrees to change the arm angle from the current setting
     * @param wristPosOffset: adjustment to the current wrist position
     * @return the Action object for use with RR
     */
    public Action GetBasketPostScore(double armOffsetDegrees, double wristPosOffset) {
        return new BasketPostScoreAction(armOffsetDegrees, wristPosOffset);
    }

    /**
     * Adjusts the arm angle and wrist position as specified on creation
     */
    public class BasketPostScoreAction implements Action {
        private final double mArmOffsetDegrees;
        private final double mWristPosOffset;
        private boolean mStarted;

        /**
         * Construct with the supplied parameters
         * @param armOffsetDegrees: adjustment to arm angle in degrees
         * @param wristPosOffset: adjustment to absolute servo position
         */
        public BasketPostScoreAction(double armOffsetDegrees, double wristPosOffset) {
            mArmOffsetDegrees = armOffsetDegrees;
            mWristPosOffset = wristPosOffset;
            mStarted = false;
        }

        /**
         * RR calls this to run
         * @param telemetryPacket: unused
         * @return: true while not complete
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!mStarted) {
                mStarted = true;
                // set the offsets
                SetArmAngleOffset(mArmOffsetDegrees);
                SetWristOffset(mWristPosOffset);
            }
            // check for complete
            return LiftBusy();
        }
    }
}
