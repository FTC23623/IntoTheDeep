package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraDrive_Manual extends HydraDrive {
    private com.qualcomm.robotcore.hardware.Gamepad mGamepad;
    public HydraDrive_Manual(HydraOpMode op, HydraImu imu) {
        super(op, imu);
        mGamepad = mOp.mDriverGamepad;
    }

    @Override
    public void Process() {
        double drive;
        double strafe;
        double rotate;
        double rotX;
        double rotY;
        double driveMaxPower;
        double sum;
        double max;
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;
        // get the yaw input from the gyro
        double yaw = 0;
        if (!mImu.Connected()) {
            mOp.mTelemetry.addData("Yaw", "disconnected");
        } else if (mImu.Calibrating()) {
            mOp.mTelemetry.addData("Yaw", "cal");
        } else {
            yaw = mImu.GetYaw();
            mOp.mTelemetry.addData("Yaw", yaw);
        }
        // Get driver controller input
        drive = mGamepad.left_stick_y;
        strafe = -mGamepad.left_stick_x * 1.1;
        if (mGamepad.cross && cFieldCentric) {
            // snap to the nearest 90 deg
            double snapHeading = yaw;
            if (yaw >= -180 && yaw <= -135) {
                snapHeading = -180;
            } else if (yaw > -135 && yaw <= -45) {
                snapHeading = -90;
            } else if (yaw > -45 && yaw <= 45) {
                snapHeading = 0;
            } else if (yaw > 45 && yaw <= 135) {
                snapHeading = 90;
            } else if (yaw > 135 && yaw <= 180) {
                snapHeading = 180;
            }
            rotate = -Math.sin((yaw - snapHeading) * Math.PI / 180) * 1.1;
        } else {
            rotate = -mGamepad.right_stick_x;
        }
        rotX = strafe * Math.cos(-yaw / 180 * Math.PI) - drive * Math.sin(-yaw / 180 * Math.PI);
        rotY = strafe * Math.sin(-yaw / 180 * Math.PI) + drive * Math.cos(-yaw / 180 * Math.PI);
        if (mGamepad.circle && cFieldCentric) {
            mImu.ResetYaw();
        }
        // Set max drive power based on driver input
        if (mGamepad.left_trigger > cTrgBtnThresh) {
            // Drive slower for better control
            driveMaxPower = cDriveSlow;
        } else if (mGamepad.right_trigger > cTrgBtnThresh) {
            // Drive faster for fun
            driveMaxPower = cDriveBoosted;
        } else {
            // Normal drive speed
            driveMaxPower = cDriveNormal;
        }
        // Scale the output power for the division we are doing later
        sum = Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate);
        if (sum > 1) {
            max = sum;
        } else {
            max = 1;
        }
        // Front left power
        if (cFieldCentric) {
            frontLeftPower = rotY + rotX;
        } else {
            frontLeftPower = drive + strafe;
        }
        frontLeftPower = frontLeftPower + rotate;
        frontLeftPower = frontLeftPower / max;
        // Rear left power
        if (cFieldCentric) {
            rearLeftPower = rotY - rotX;
        } else {
            rearLeftPower = drive - strafe;
        }
        rearLeftPower = rearLeftPower + rotate;
        rearLeftPower = rearLeftPower / max;
        // Front right power
        if (cFieldCentric) {
            frontRightPower = rotY - rotX;
        } else {
            frontRightPower = drive - strafe;
        }
        frontRightPower = frontRightPower - rotate;
        frontRightPower = frontRightPower / max;
        // Rear right power
        if (cFieldCentric) {
            rearRightPower = rotY + rotX;
        } else {
            rearRightPower = drive + strafe;
        }
        rearRightPower = rearRightPower - rotate;
        rearRightPower = rearRightPower / max;
        // Set power to the motors
        mMotDrBkLt.setPower(rearLeftPower * driveMaxPower);
        mMotDrBkRt.setPower(rearRightPower * driveMaxPower);
        mMotDrFrLt.setPower(frontLeftPower * driveMaxPower);
        mMotDrFrRt.setPower(frontRightPower * driveMaxPower);
        mOp.mTelemetry.addData("LeftFront", frontLeftPower);
        mOp.mTelemetry.addData("RightFront", frontRightPower);
        mOp.mTelemetry.addData("LeftRear", rearLeftPower);
        mOp.mTelemetry.addData("RightRear", rearRightPower);
    }
}
