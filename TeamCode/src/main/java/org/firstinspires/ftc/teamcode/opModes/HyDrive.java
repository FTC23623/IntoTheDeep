package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Drive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Hub;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lens;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Config
@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {
  private HydraOpMode mOpMode;
  private Imu mImu;
  private Drive mDrive;
  // private Lens mLens;
  // private Lights mLights;
  private Arm mArm;
  private Intake mIntake;
  private ElapsedTime mLoopSleep;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // Initialization Routines
    final ElementTypes elementType = ElementTypes.Specimen;
    mLoopSleep = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1,
            gamepad2, elementType);
    mImu = new Imu_Hub(mOpMode);
    mDrive = new Drive_Manual(mOpMode, mImu);
    // mLens = new Lens(mOpMode);
    // mLights = new Lights(mOpMode);
    mArm = new Arm(mOpMode);
    mIntake = new Intake(mOpMode);
    while (!mImu.Connected() || mImu.Calibrating()) {
      if (isStopRequested() || !opModeIsActive()) {
        break;
      }
    }
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    telemetry.addData("Auto Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    waitForStart();
    mLoopSleep.reset();
    while (opModeIsActive()) {
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      if (mArm.Startup(false)) {
        break;
      }
      mLoopSleep.reset();
      idle();
    }
    mLoopSleep.reset();
    while (opModeIsActive()) {
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      // Pass user input to the systems
      mArm.HandleUserInput();
      mIntake.HandleUserInput();
      // System processes
      mDrive.Process();
      // mLights.SetColor(mLens.GetDetectedSample());
      mArm.Process();
      mIntake.Process();
      // Update telemetry once for all processes
      telemetry.update();
      mLoopSleep.reset();
      idle();
    }
  }
}