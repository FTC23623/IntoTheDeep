package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.ArmActions;
import org.firstinspires.ftc.teamcode.types.ArmMoveStates;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.IntakeStates;

@Config
public class Intake {
    private HydraOpMode mOp;
    private Servo mServo;
    private ElapsedTime mTimeSinceHaveElement;
    private ColorRangeSensor mSensor;
    private IntakeStates mState;
    private double mServoPower;
    private boolean mRunIn;
    private boolean mRunOut;
    private final double mElementDetectionDistance = 0.7;

    public Intake(HydraOpMode opmode) {
        mOp = opmode;
        mServo = mOp.mHardwareMap.get(Servo.class, "intakeServo");
        mSensor = mOp.mHardwareMap.get(ColorRangeSensor.class, "intakeColorSensor");
        mServoPower = Constants.contServoOff;
        mState = IntakeStates.Idle;
        mTimeSinceHaveElement = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mRunIn = false;
        mRunOut = false;
    }

    /**
     * Take user input for next process call
     */
    public void HandleUserInput() {
        mRunIn = mOp.mOperatorGamepad.right_trigger > Constants.trgBtnThresh;
        mRunOut = mOp.mOperatorGamepad.left_trigger > Constants.trgBtnThresh;
    }

    /**
     * Start running the intake in
     */
    public void RunIn() {
        mRunIn = true;
        mRunOut = false;
    }

    /**
     * Start running the intake out
     */
    public void RunOut() {
        mRunIn = false;
        mRunOut = true;
    }

    public void Stop() {
        mRunIn = false;
        mRunOut = false;
        mState = IntakeStates.Idle;
    }

    /**
     * Process for running the intake
     */
    public void Process() {
        // use this to exit directly to out regardless of the state
        if (mRunOut && !mRunIn) {
            mState = IntakeStates.Out;
        }
        switch (mState) {
            case Idle:
                // if we want to run in, start the servo and change state
                // otherwise keep the servo off
                if (mRunIn) {
                    mState = IntakeStates.In;
                } else {
                    mServoPower = Constants.contServoOff;
                    break;
                }
                // fallthrough
            case In:
                // keep the servo running
                mServoPower = Constants.contServoForward;
                if (HaveElement()) {
                    // we have detected an element in the intake
                    // transition to the detected state and start our timer
                    mState = IntakeStates.InDetected;
                    mTimeSinceHaveElement.reset();
                }
                break;
            case InDetected:
                // keep running for a bit to make sure the element is fully grasped
                if (mTimeSinceHaveElement.milliseconds() > 500) {
                    // time has elapsed, go to hold
                    mState = IntakeStates.Hold;
                    mRunIn = false;
                }
                break;
            case Hold:
                // keep the servo off until it's time to release it
                if (mRunIn) {
                    mState = IntakeStates.In;
                } else if (mRunOut) {
                    // score it!
                    mState = IntakeStates.Out;
                    mServoPower = Constants.contServoBackward;
                } else if (!HaveElement()) {
                    mState = IntakeStates.Idle;
                } else {
                    mServoPower = Constants.contServoOff;
                }
                break;
            case Out:
                mServoPower = Constants.contServoBackward;
                if (!mRunOut) {
                    mState = IntakeStates.Idle;
                }
                break;
        }
        // setting position on continuous rotation sets the power and direction
        mServo.setPosition(mServoPower);
        // get the distance from the distance sensor for telemetry
        double distance = mSensor.getDistance(DistanceUnit.INCH);
        mOp.mTelemetry.addData("Distance", distance);
        // get the color from the distance sensor for telemetry
        //NormalizedRGBA color = mSensor.getNormalizedColors();
        //mOp.mTelemetry.addData("Red", color.red);
        //mOp.mTelemetry.addData("Green", color.green);
        //mOp.mTelemetry.addData("Blue", color.blue);
    }

    /**
     * Returns whether we have an element in the intake
     * @return true if an element is detected
     */
    public boolean HaveElement() {
        double distance = mSensor.getDistance(DistanceUnit.INCH);
        return distance < mElementDetectionDistance;
    }

    /*
     * ROAD RUNNER API
     */

    /**
     * Runs the intake until an element has been picked up
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetAction(IntakeActions action) {
        return new RunAction(action);
    }
    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private boolean started = false;
        // run has been called once
        private final IntakeActions mAction;

        // construct on the supplied action
        public RunAction(IntakeActions action) {
            mAction = action;
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (mAction) {
                case IntakeElement:
                    if (!started) {
                        RunIn();
                        started = true;
                    }
                    Process();
                    return !HaveElement();
                case InStart:
                    RunIn();
                    Process();
                    return mState != IntakeStates.In;
                case OutContinuous:
                    RunOut();
                    Process();
                    return mState != IntakeStates.Out;
                case Stop:
                    Stop();
                    Process();
                    return false;
                default:
                    return false;
            }
        }
    }
}
