package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Drive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Hub;
import org.firstinspires.ftc.teamcode.subsystems.Imu_navx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lens;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArm;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

import java.util.List;

@Config
@Disabled
@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {
  private HydraOpMode mOpMode;
  private Imu mImu;
  private Drive mDrive;
  // private Lens mLens;
  // private Lights mLights;
  private Arm mArm;
  private Intake mIntake;
  private Claw mClaw;
  private SpecimenArm mSpecArm;
  private ElapsedTime mLoopSleep;
  protected ElementTypes mStartElementType = ElementTypes.Sample;
  private Debouncer mDriverTriangle;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // Initialization Routines
    mLoopSleep = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1,
            gamepad2, mStartElementType);
    mImu = new Imu_navx(mOpMode);
    mDrive = new Drive_Manual(mOpMode, mImu);
    // mLens = new Lens(mOpMode);
    // mLights = new Lights(mOpMode);
    mArm = new Arm(mOpMode, false);
    mIntake = new Intake(mOpMode);
    mClaw = new Claw(mOpMode);
    mSpecArm = new SpecimenArm(mOpMode);
    mDriverTriangle = new Debouncer(9);
    while (!mImu.Connected() || mImu.Calibrating()) {
      if (isStopRequested() || !opModeIsActive()) {
        break;
      }
    }
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    telemetry.addData("Auto Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    waitForStart();
    mLoopSleep.reset();
    while (opModeIsActive()) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      if (mArm.Startup(false)) {
        break;
      }
      mLoopSleep.reset();
      idle();
    }
    mLoopSleep.reset();
    while (opModeIsActive()) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      // Pass user input to the systems
      mArm.HandleUserInput();
      mIntake.HandleUserInput();
      mClaw.HandleUserInput();
      mSpecArm.HandleUserInput();
      HandleElementSwitch();
      // System processes
      mDrive.Process();
      // mLights.SetColor(mLens.GetDetectedSample());
      if (mArm.Process()) {
        mIntake.RunIn();
      }
      mIntake.Process();
      mSpecArm.Process();
      // Update telemetry once for all processes
      telemetry.update();
      mLoopSleep.reset();
      idle();
    }
  }

  /**
   * Allows the driver to change the target element mid-opmode by holding triangle
   * Rumble feedback on change
   */
  private void HandleElementSwitch() {
    // debounce the button press
    mDriverTriangle.In(gamepad1.triangle);
    if (mDriverTriangle.Out()) {
      mDriverTriangle.Used();
      // we want to change our target element
      switch (mOpMode.mTargetElement) {
        case Sample:
          mOpMode.mTargetElement = ElementTypes.Specimen;
          break;
        case Specimen:
          mOpMode.mTargetElement = ElementTypes.Sample;
          break;
      }
      gamepad1.rumbleBlips(3);
    }
    telemetry.addData("Type", mOpMode.mTargetElement);
  }
}