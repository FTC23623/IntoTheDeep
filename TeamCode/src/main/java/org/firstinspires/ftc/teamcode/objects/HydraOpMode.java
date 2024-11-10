package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.datalogger.DriveDatalogger;
import org.firstinspires.ftc.teamcode.datalogger.ObjDetDatalogger;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

public class HydraOpMode {
    public Telemetry mTelemetry;
    public HardwareMap mHardwareMap;
    public DriveDatalogger mDriveLogger;
    public ObjDetDatalogger mObjLogger;
    public com.qualcomm.robotcore.hardware.Gamepad mDriverGamepad;
    public com.qualcomm.robotcore.hardware.Gamepad mOperatorGamepad;
    public ElementTypes mTargetElement;
    public double mLoopTime;
    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap, DriveDatalogger driveLogger,
                       ObjDetDatalogger objDetDatalogger, com.qualcomm.robotcore.hardware.Gamepad driverGamepad,
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
