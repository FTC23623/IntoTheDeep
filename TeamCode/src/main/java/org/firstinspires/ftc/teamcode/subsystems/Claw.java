package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.ClawActions;
import org.firstinspires.ftc.teamcode.types.Constants;

public class Claw {
    private final HydraOpMode mOp;
    private final Servo mServo;
    // servo positions
    private final double mClosed = 0.52;
    private final double mOpen = 0.4;
    // button debouncers
    private final Debouncer mRightBumperPress;
    private final Debouncer mRightBumperRelease;

    /**
     * Construct and intialize a new Claw
     * @param opmode
     */
    public Claw(HydraOpMode opmode) {
        mOp = opmode;
        mServo = mOp.mHardwareMap.get(Servo.class, "clawServo");
        mRightBumperPress = new Debouncer(Constants.debounce);
        mRightBumperRelease = new Debouncer(Constants.debounce);
    }

    /**
     * Close the claw
     */
    public void Close() {
        mServo.setPosition(mClosed);
    }

    /**
     * Open the claw
     */
    public void Open() {
        mServo.setPosition(mOpen);
    }

    /**
     * Handle user input for the claw
     */
    public void HandleUserInput() {
        // these two debouncers are inverted so there is some hysteresis between open and close
        mRightBumperPress.In(mOp.mOperatorGamepad.right_bumper);
        mRightBumperRelease.In(!mOp.mOperatorGamepad.right_bumper);
        // open or close the claw based on input
        if (mRightBumperPress.Out()) {
            Open();
        } else if (mRightBumperRelease.Out()) {
            Close();
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
    public Action GetAction(ClawActions action) {
        return new RunAction(action);
    }

    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private final ClawActions mRRAction;

        // construct on the supplied action
        public RunAction(ClawActions action) {
            mRRAction = action;
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (mRRAction) {
                case Close:
                    Close();
                    break;
                case Open:
                    Open();
                    break;
            }
            // "instantaneous." always return false
            return false;
        }
    }
}