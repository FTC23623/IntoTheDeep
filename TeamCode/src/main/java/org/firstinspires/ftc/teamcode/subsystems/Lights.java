package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.HydraSampleColor;

import java.util.HashMap;
import java.util.Map;

public class Lights {
    HydraOpMode mOp;
    RevBlinkinLedDriver mBlinkin;
    Map<HydraSampleColor, RevBlinkinLedDriver.BlinkinPattern> mPatterns;

    public Lights(HydraOpMode op) {
        mOp = op;
        mBlinkin = mOp.mHardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        mPatterns = new HashMap<HydraSampleColor, RevBlinkinLedDriver.BlinkinPattern>();
        mPatterns.put(HydraSampleColor.SampleColorNone, RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
        mPatterns.put(HydraSampleColor.SampleColorRed, RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        mPatterns.put(HydraSampleColor.SampleColorBlue, RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        mPatterns.put(HydraSampleColor.SampleColorYellow, RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        mBlinkin.setPattern(mPatterns.get(HydraSampleColor.SampleColorNone));
    }

    public void SetColor(HydraSampleColor sample) {
        if (mPatterns.containsKey(sample)) {
            mBlinkin.setPattern(mPatterns.get(sample));
        } else {
            mBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }
    }
}
