package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.Constants;

public class Claw {
    private final HydraOpMode mOp;
    private final Servo mServo;
    private final double mClosed = 0.52;
    private final double mOpen = 0.4;
    private final Debouncer mRightBumperPress;
    private final Debouncer mRightBumperRelease;

    public Claw(HydraOpMode opmode) {
        mOp = opmode;
        mServo = mOp.mHardwareMap.get(Servo.class, "clawServo");
        mRightBumperPress = new Debouncer(Constants.debounce);
        mRightBumperRelease = new Debouncer(Constants.debounce);
    }

    public void Close() {
        mServo.setPosition(mClosed);
    }

    public void Open() {
        mServo.setPosition(mOpen);
    }

    public void HandleUserInput() {
        mRightBumperPress.In(mOp.mOperatorGamepad.right_bumper);
        mRightBumperRelease.In(!mOp.mOperatorGamepad.right_bumper);
        if (mRightBumperPress.Out()) {
            Open();
        } else if (mRightBumperRelease.Out()) {
            Close();
        }
    }
}