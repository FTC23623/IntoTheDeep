package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.HydraDrive;
import org.firstinspires.ftc.teamcode.subsystems.HydraDrive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.HydraImu;
import org.firstinspires.ftc.teamcode.subsystems.HydraImu_Hub;
import org.firstinspires.ftc.teamcode.subsystems.HydraImu_navx;

@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {
  private HydraOpMode mOpMode;
  private HydraImu mImu;
  private HydraDrive mDrive;

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
    mImu = new HydraImu_Hub(mOpMode);
    mDrive = new HydraDrive_Manual(mOpMode, mImu);
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
      // Update telemetry once for all processes
      telemetry.update();
      sleep(20);
    }
  }
}