package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.HydraConstants;
import org.firstinspires.ftc.teamcode.types.IntakeStates;

@Config
public class Intake {
    private HydraOpMode mOp;
    private Servo mServo;
    private ElapsedTime mTimeSinceHaveElement;
    private ColorRangeSensor mSensor;
    private IntakeStates mState;
    public static double mServoPower;

    public Intake(HydraOpMode opmode) {
        mOp = opmode;
        mServo = mOp.mHardwareMap.get(Servo.class, "intakeServo");
        mSensor = mOp.mHardwareMap.get(ColorRangeSensor.class, "intakeColorSensor");
        mServoPower = HydraConstants.contServoOff;
        mState = IntakeStates.IntakeSt_Idle;
        mTimeSinceHaveElement = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void Process() {
        boolean operateIn = mOp.mOperatorGamepad.right_trigger > HydraConstants.trgBtnThresh;
        boolean operateOut = mOp.mOperatorGamepad.left_trigger > HydraConstants.trgBtnThresh;
        if (operateOut && !operateIn) {
            mState = IntakeStates.IntakeSt_Out;
        }
        switch (mState) {
            case IntakeSt_Idle:
                if (operateIn) {
                    mState = IntakeStates.IntakeSt_In;
                    mServoPower = HydraConstants.contServoForward;
                } else {
                    mServoPower = HydraConstants.contServoOff;
                }
                break;
            case IntakeSt_In:
                mServoPower = HydraConstants.contServoForward;
                if (HaveElement()) {
                    mState = IntakeStates.IntakeSt_InDetected;
                    mTimeSinceHaveElement.reset();
                }
                break;
            case IntakeSt_InDetected:
                if (mTimeSinceHaveElement.milliseconds() > 200) {
                    mState = IntakeStates.IntakeSt_Hold;
                }
                break;
            case IntakeSt_Hold:
                if (operateOut) {
                    mState = IntakeStates.IntakeSt_Out;
                    mServoPower = HydraConstants.contServoBackward;
                } else {
                    mServoPower = HydraConstants.contServoOff;
                }
                break;
            case IntakeSt_Out:
                mState = IntakeStates.IntakeSt_Idle;
                mServoPower = HydraConstants.contServoBackward;
                break;
        }

        mServo.setPosition(mServoPower);
        double distance = mSensor.getDistance(DistanceUnit.INCH);
        mOp.mTelemetry.addData("Distance", distance);
        NormalizedRGBA color = mSensor.getNormalizedColors();
        mOp.mTelemetry.addData("Color", color);
    }

    public boolean HaveElement() {
        double distance = mSensor.getDistance(DistanceUnit.INCH);
        if (distance < 0.7) {
            return true;
        }
        return false;
    }
}
