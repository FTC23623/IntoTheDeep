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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Config
public class SpecimenArm {
    private final HydraOpMode mOp;
    private final DcMotorEx mLiftMotor;
    private final PIDController mSpecArmPid;
    private final RevTouchSensor mLiftPickupPosition;
    // PID coefficients
    private final double mSpecArmP = 0.015;
    private final double mSpecArmI = 0.0;
    private final double mSpecArmD = 0.001;
    // Target in degrees
    private int mLiftTargetTicks;
    // REV Core Hex
    // 288 ticks per rotation
    private final double mTicksPerDegree = 288.0 / 360.0;
    // offset to apply to the motor ticks
    private int mStartupTicksOffset;
    // offset of the kickstand position in ticks
    private final int mKickstandTicksOffset = 17;
    // current arm action
    private ArmActions mAction;
    // angle of the arm when the opmode starts
    private final double mStartAngle = 8.0;
    // angles at action positions
    private final double mHighScorePosition = 117.0;
    private final double mScorePosition = 35.0;
    private final double mPickupPosition = mStartAngle;
    private final double mOffStandPosition = 90.0;
    public static int mClawReleaseTime = 150;
    // button debouncers
    private final Debouncer mSquare;
    private final Debouncer mTriangle;
    private final Debouncer mDpadDown;
    private final Debouncer mDpadLeft;
    private ElapsedTime mClawOpenTimer;
    private boolean mClawOpen;
    private boolean mStartup;

    /**
     * Construct and initialize a new SpecimenArm
     * @param opMode
     */
    public SpecimenArm(HydraOpMode opMode) {
        mOp = opMode;
        mLiftMotor = mOp.mHardwareMap.get(DcMotorEx.class, "specArmMotor");
        mLiftPickupPosition = mOp.mHardwareMap.get(RevTouchSensor.class, "specArmPickupSwitch");
        mSpecArmPid = new PIDController(mSpecArmP, mSpecArmI, mSpecArmD);
        mSquare = new Debouncer(Constants.debounce);
        mTriangle = new Debouncer(Constants.debounce);
        mDpadDown = new Debouncer(Constants.debounce);
        mDpadLeft = new Debouncer(Constants.debounce);
        mAction = ArmActions.Idle;
        mLiftTargetTicks = 0;
        mLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mStartupTicksOffset = 0;
        mClawOpenTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mClawOpen = false;
        mStartup = false;
    }

    /**
     * Call after init and before start to set the start position of the arm
     */
    public boolean Startup() {
        if (!mStartup) {
            if (AtPickup()) {
                // arm is at the pickup location, no offset is needed
                mStartupTicksOffset = 0;
            } else {
                mStartupTicksOffset = mKickstandTicksOffset;
            }
            mStartup = true;
        }
        return mStartup;
    }

    /**
     * Process input from the operator controllers
     */
    public void HandleUserInput() {
        // pass button presses into the debouncer
        mSquare.In(mOp.mOperatorGamepad.square);
        mTriangle.In(mOp.mOperatorGamepad.triangle);
        mDpadDown.In(mOp.mOperatorGamepad.dpad_down);
        mDpadLeft.In(mOp.mOperatorGamepad.dpad_left);
        // handle buttons that are pressed
        if (mSquare.Out()) {
            mSquare.Used();
            SetAction(ArmActions.RunPickup);
        } else if (mTriangle.Out()) {
            mTriangle.Used();
            if (mOp.mTargetElement == ElementTypes.Specimen) {
                SetAction(ArmActions.RunScoreHigh);
            }
        } else if (mDpadDown.Out()) {
            mDpadDown.Used();
            if (mOp.mTargetElement == ElementTypes.Specimen) {
                SetAction(ArmActions.RunScoreHighScore);
            }
        } else if (mDpadLeft.Out()) {
            mDpadLeft.Used();
            SetAction(ArmActions.RunHome);
        }
    }

    /**
     * Set the desired action for the arm
     * @param action: The action to perform
     */
    public void SetAction(ArmActions action) {
        mAction = action;
        switch (action) {
            case RunPickup:
                SetAngle(mPickupPosition);
                break;
            case RunScoreHigh:
                SetAngle(mHighScorePosition);
                break;
            case RunScoreHighScore:
                mClawOpenTimer.reset();
                mClawOpen = true;
                SetAngle(mScorePosition);
                break;
            case RunScoreLow:
                SetAngle(mOffStandPosition);
                break;
            case RunHome:
                SetAngle(mPickupPosition);
                break;
            default:
                break;
        }
    }

    /**
     * Set the target angle for the arm
     * @param angle: The target angle in degrees
     */
    public void SetAngle(double angle) {
        mLiftTargetTicks = (int)((angle - mStartAngle) * mTicksPerDegree - mStartupTicksOffset);
    }

    /**
     * Manage the PID loop for the arm
     */
    public boolean Process() {
        // set to idle when we're near position in case something is waiting
        if (!LiftBusy()) {
            if (mAction == ArmActions.RunHome && !AtPickup()) {
                mLiftTargetTicks -= 6;
                mStartupTicksOffset = -mLiftTargetTicks;
            } else {
                mAction = ArmActions.Idle;
            }
        }
        double pid = 0;
        if (AtPickup() && mLiftTargetTicks == -mStartupTicksOffset) {
            mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSpecArmPid.reset();
            mLiftMotor.setPower(0);
            mStartupTicksOffset = 0;
            SetAngle(mPickupPosition);
        } else {
            // get the current position to calculate error
            int currentPos = mLiftMotor.getCurrentPosition();
            mSpecArmPid.setPID(mSpecArmP, mSpecArmI, mSpecArmD);
            // calculate pid for power and run
            pid = mSpecArmPid.calculate(currentPos, mLiftTargetTicks);
            mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mLiftMotor.setPower(pid);
        }
        mOp.mTelemetry.addData("Spec Tgt", mLiftTargetTicks);
        mOp.mTelemetry.addData("Spec Pos", mLiftMotor.getCurrentPosition());
        mOp.mTelemetry.addData("Spec Pwr", pid);
        mOp.mTelemetry.addData("Spec Curr", mLiftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        mOp.mTelemetry.addData("Spec Pickup", AtPickup());
        mOp.mTelemetry.addData("Spec Action", mAction);
        mOp.mTelemetry.addData("Spec Tgt Offset", mStartupTicksOffset);
        if (mClawOpen && mClawOpenTimer.milliseconds() >= mClawReleaseTime) {
            mClawOpen = false;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns whether or not the arm is still moving to position
     * @return true when near the target position
     */
    public boolean LiftBusy() {
        return Math.abs(mLiftMotor.getCurrentPosition() - mLiftTargetTicks) > 10;
    }

    private boolean AtPickup() {
        return mLiftPickupPosition.isPressed();
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
        private final ArmActions mRRAction;
        // run has been called once
        private boolean started = false;
        // timeout for scoring position because of the claw
        private final ElapsedTime mScoreTimeout;

        // construct on the supplied action
        public RunAction(ArmActions action) {
            mRRAction = action;
            mScoreTimeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                mScoreTimeout.reset();
                switch (mRRAction) {
                    case RunPickup:
                    case RunScoreHigh:
                    case RunScoreHighScore:
                    case RunScoreLow:
                        SetAction(mRRAction);
                        break;
                    default:
                        SetAction(ArmActions.Idle);
                        break;
                }
                started = true;
            }
            if (mRRAction == ArmActions.RunScoreHighScore) {
                return mScoreTimeout.milliseconds() < mClawReleaseTime;
            } else {
                return mAction != ArmActions.Idle;
            }
        }
    }
}
