package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Drive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Hub;
import org.firstinspires.ftc.teamcode.subsystems.Lens;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

@TeleOp(name = "HyDrive-Java")
public class HyDrive extends LinearOpMode {
  private HydraOpMode mOpMode;
  private Imu mImu;
  private Drive mDrive;
  private Lens mLens;
  private Lights mLights;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // Initialization Routines
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1,
            gamepad2);
    mImu = new Imu_Hub(mOpMode);
    mDrive = new Drive_Manual(mOpMode, mImu);
    mLens = new Lens(mOpMode);
    mLights = new Lights(mOpMode);
    while (!mImu.Connected() || mImu.Calibrating()) {
      if (isStopRequested() || !opModeIsActive()) {
        break;
      }
    }
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    telemetry.addData("Auton Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      // System processes
      mDrive.Process();
      mLights.SetColor(mLens.GetDetectedSample());
      // Update telemetry once for all processes
      telemetry.update();
      sleep(20);
    }
  }
}