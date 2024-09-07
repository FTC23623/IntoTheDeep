package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraImu_navx implements HydraImu {
    protected AHRS mNavx;
    protected double mOffset;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    public HydraImu_navx(HydraOpMode opMode) {
        mNavx = AHRS.getInstance(opMode.mHardwareMap.get(NavxMicroNavigationSensor.class,
                "navx"), AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        mOffset = 0;
    }

    public boolean Connected() {
        return mNavx.isConnected();
    }

    public void ResetYaw() {
        mNavx.zeroYaw();
        mOffset = 0;
    }

    public double GetYaw() {
        double ret = mNavx.getYaw() * -1;
        return mOffset + ret;
    }

    public void SetYawOffset(double offset) {
        mOffset = offset;
    }

    public boolean Calibrating() {
        return mNavx.isCalibrating();
    }

    public void Close() {
        mNavx.close();
    }
}
