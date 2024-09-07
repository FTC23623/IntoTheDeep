package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraImu_Hub implements HydraImu {
    protected IMU imu;
    protected double mOffset;

    public HydraImu_Hub(HydraOpMode opMode) {
        // Initialization Routines
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu = opMode.mHardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        mOffset = 0;
    }

    public void ResetYaw() {
        imu.resetYaw();
        mOffset = 0;
    }

    public double GetYaw() {
        double ret = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return ret + mOffset;
    }

    public void SetYawOffset(double offset) {
        mOffset = offset;
    }

    public boolean Calibrating() {
        return false;
    }

    public boolean Connected() {
        return true;
    }

    public void Close() {
    }
}
