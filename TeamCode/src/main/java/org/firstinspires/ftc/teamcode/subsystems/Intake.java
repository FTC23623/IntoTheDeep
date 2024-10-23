package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.Constants;
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
                    mServoPower = Constants.contServoForward;
                } else {
                    mServoPower = Constants.contServoOff;
                }
                break;
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
                if (mTimeSinceHaveElement.milliseconds() > 200) {
                    // time has elapsed, go to hold
                    mState = IntakeStates.Hold;
                }
                break;
            case Hold:
                // keep the servo off until it's time to release it
                if (mRunOut) {
                    // score it!
                    mState = IntakeStates.Out;
                    mServoPower = Constants.contServoBackward;
                } else {
                    mServoPower = Constants.contServoOff;
                }
                break;
            case Out:
                // keep running out until the element is no longer detected
                // TODO: do we need to use the timer here?
                if (HaveElement()) {
                    mServoPower = Constants.contServoBackward;
                } else {
                    mServoPower = Constants.contServoOff;
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
        NormalizedRGBA color = mSensor.getNormalizedColors();
        mOp.mTelemetry.addData("Red", color.red);
        mOp.mTelemetry.addData("Green", color.green);
        mOp.mTelemetry.addData("Blue", color.blue);
    }

    /**
     * Returns whether we have an element in the intake
     * @return true if an element is detected
     */
    public boolean HaveElement() {
        double distance = mSensor.getDistance(DistanceUnit.INCH);
        return distance < mElementDetectionDistance;
    }
}
