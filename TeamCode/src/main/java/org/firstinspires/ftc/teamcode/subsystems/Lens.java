package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.HydraSampleColor;

public class Lens {
    private HuskyLens mHusky;
    private HydraOpMode mOp;
    private int mMaxX = 320;
    private int mMaxY = 240;

    public Lens(HydraOpMode op) {
        mOp = op;
        mHusky = mOp.mHardwareMap.get(HuskyLens.class, "huskylens");
        mHusky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public HydraSampleColor GetDetectedSample() {
        HuskyLens.Block[] blocks = mHusky.blocks();
        // if nothing is detected, leave now
        if (blocks.length == 0) {
            return HydraSampleColor.SampleColorNone;
        }
        // position
        // x/y of the center of the object
        // x increases from left to right
        // y increases from top to bottom
        int closestToCenter = 0;
        int closestToCenterDistance = DistanceFromCenter(blocks[0]);
        int closestToUs = 0;
        int closestToUsDistance = DistanceFromUs(blocks[0]);
        for (int i = 1; i < blocks.length; i++) {
            int distCenter = DistanceFromCenter(blocks[i]);
            if (distCenter < closestToCenterDistance) {
                closestToCenter = i;
                closestToCenterDistance = distCenter;
            }
            int distUs = DistanceFromUs(blocks[i]);
            if (distUs < closestToUsDistance) {
                closestToUs = i;
                closestToUsDistance = distUs;
            }
        }
        return GetColorFromHuskyId(blocks[closestToCenter].id);
    }

    private HydraSampleColor GetColorFromHuskyId(int id) {
        switch (id) {
            case 1:
                return HydraSampleColor.SampleColorBlue;
            case 2:
                return HydraSampleColor.SampleColorRed;
            case 3:
                return HydraSampleColor.SampleColorYellow;
            default:
                return HydraSampleColor.SampleColorNone;
        }
    }

    private int DistanceFromCenter(HuskyLens.Block block) {
        return Math.abs(block.x - mMaxX / 2);
    }

    private int DistanceFromUs(HuskyLens.Block block) {
        return Math.abs(mMaxY - block.y);
    }
}
