package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.datalogger.HydraDriveDatalogger;
import org.firstinspires.ftc.teamcode.datalogger.HydraObjDetDatalogger;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

public class HydraOpMode {
    public Telemetry mTelemetry;
    public HardwareMap mHardwareMap;
    public HydraDriveDatalogger mDriveLogger;
    public HydraObjDetDatalogger mObjLogger;
    public com.qualcomm.robotcore.hardware.Gamepad mDriverGamepad;
    public com.qualcomm.robotcore.hardware.Gamepad mOperatorGamepad;
    public ElementTypes mTargetElement;
    public double mLoopTime;
    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap, HydraDriveDatalogger driveLogger,
                       HydraObjDetDatalogger objDetDatalogger, com.qualcomm.robotcore.hardware.Gamepad driverGamepad,
                       com.qualcomm.robotcore.hardware.Gamepad operatorGamepad, ElementTypes targetElement) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mDriveLogger = driveLogger;
        mObjLogger = objDetDatalogger;
        mDriverGamepad = driverGamepad;
        mOperatorGamepad = operatorGamepad;
        mTargetElement = targetElement;
        mLoopTime = 0;
    }
}
