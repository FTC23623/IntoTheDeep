package org.firstinspires.ftc.teamcode.objects;

public class OpmodeHeading {
    static private double mYawOffset = 0;

    public static void SetOffset(double offset) {
        mYawOffset = offset;
    }

    public static double GetOffset() {
        return mYawOffset;
    }
}
