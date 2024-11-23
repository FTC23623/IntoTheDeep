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
    public final double mSpecArmP = 0.022;
    public final double mSpecArmI = 0.0;
    public final double mSpecArmD = 0.0018;
    // Target in degrees
    private double mLiftTargetTicks;
    // REV Core Hex
    // 288 ticks per rotation
    private final double mTicksPerDegree = 288.0 / 360.0;
    // current arm action
    private ArmActions mAction;
    // angle of the arm when the opmode starts
    private final double mStartAngle = 8.0;
    // angles at action positions
    private final double mHighScorePosition = 117.0;
    private final double mScorePosition = 35.0;
    private final double mPickupPosition = mStartAngle;
    // button debouncers
    private final Debouncer mSquare;
    private final Debouncer mTriangle;
    private final Debouncer mDpadDown;

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
        mAction = ArmActions.Idle;
        mLiftTargetTicks = 0;
        mLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Process input from the operator controllers
     */
    public void HandleUserInput() {
        // pass button presses into the debouncer
        mSquare.In(mOp.mOperatorGamepad.square);
        mTriangle.In(mOp.mOperatorGamepad.triangle);
        mDpadDown.In(mOp.mOperatorGamepad.dpad_down);
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
                SetAngle(mScorePosition);
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
        mLiftTargetTicks = (angle - mStartAngle) * mTicksPerDegree;
    }

    /**
     * Manage the PID loop for the arm
     */
    public void Process() {
        // set to idle when we're near position in case something is waiting
        if (!LiftBusy()) {
            mAction = ArmActions.Idle;
        }
        double pid = 0;
        if (AtPickup() && mLiftTargetTicks == 0) {
            mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSpecArmPid.reset();
            mLiftMotor.setPower(0);
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
    }

    /**
     * Returns whether or not the arm is still moving to position
     * @return true when near the target position
     */
    public boolean LiftBusy() {
        return Math.abs(mLiftMotor.getCurrentPosition() - mLiftTargetTicks) < 3;
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
                        SetAction(mRRAction);
                        break;
                    default:
                        SetAction(ArmActions.Idle);
                        break;
                }
                started = true;
            }
            Process();
            if (mAction == ArmActions.RunScoreHighScore) {
                return mScoreTimeout.milliseconds() > 250;
            } else {
                return mAction != ArmActions.Idle;
            }
        }
    }
}
