package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.SampleColor;

import java.util.HashMap;
import java.util.Map;

public class Lights {
    HydraOpMode mOp;
    RevBlinkinLedDriver mBlinkin;
    Map<SampleColor, RevBlinkinLedDriver.BlinkinPattern> mPatterns;

    public Lights(HydraOpMode op) {
        mOp = op;
        mBlinkin = mOp.mHardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        mPatterns = new HashMap<SampleColor, RevBlinkinLedDriver.BlinkinPattern>();
        mPatterns.put(SampleColor.None, RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
        mPatterns.put(SampleColor.Red, RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        mPatterns.put(SampleColor.Blue, RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        mPatterns.put(SampleColor.Yellow, RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        mBlinkin.setPattern(mPatterns.get(SampleColor.None));
    }

    public void SetColor(SampleColor sample) {
        if (mPatterns.containsKey(sample)) {
            mBlinkin.setPattern(mPatterns.get(sample));
        } else {
            mBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }
    }
}
