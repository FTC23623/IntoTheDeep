package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.subsystems.HydraImu;
import org.firstinspires.ftc.teamcode.subsystems.HydraImu_navx;

@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {
  private HydraImu mImu;
  private DcMotor MotLwrArm;
  private DcMotor MotUprArm;
  private Servo SrvPxlPos1;
  private Servo SrvPxlPos2;
  private ColorSensor SenColPxlPos1;
  private ColorSensor SenColPxlPos2;
  private DcMotor MotDrFrLt;
  private DcMotor MotDrBkLt;
  private DcMotor MotDrFrRt;
  private DcMotor MotDrBkRt;
  private DcMotor MotPxlIntk;
  private Servo SrvDrLnch;
  private LED LED4;
  private LED LED3;
  private LED LED2;
  private LED LED1;
  private DistanceSensor SenColPxlPos1_DistanceSensor;
  private DistanceSensor SenColPxlPos2_DistanceSensor;
  int armPositionState;
  boolean allowManualArmControl;
  int triangleButtonPress;
  long droneLaunchTime;
  double cTrgBtnThresh;
  double cDroneServoStop;
  double cDroneServoLaunch;
  double cLowerArmAutoMotorPwr;
  double cUpperArmAutoMotorPwr;
  int cDroneLaunchDuration;
  int cPixelPos1Dist;
  int cPixelPos2Dist;
  double cArmMotorDB;
  boolean cAllowFrontScoreFromCassette;
  double cCasStop;
  int cDriveBoosted;
  List cLowerArmPositions;
  int cIntakeStop;
  int cCasBackToFront;
  int cCasFrontToBack;
  double cDriveNormal;
  List cUpperArmPositions;
  double cDriveSlow;
  List cArmPositionNames;
  int cIntakeIn;
  int cIntakeOut;
  double cLowerArmManualMotorPwrScale;
  boolean cFieldCentric;
  double cUpperArmManualMotorPwrScale;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int cLowerArmPos0Home;
    int cUpperArmPos0Home;
    int cLowerArmPos1LiftBox;
    int cUpperArmPos1LiftBox;
    int cLowerArmPos2LiftArm;
    int cUpperArmPos2LiftArm;
    int cLowerArmPos3BackScore;
    int cUpperArmPos3BackScore;
    int cLowerArmPos4FrontScore;
    int cUpperArmPos4FrontScore;
    int cLowerArmPos5Hang;
    int cUpperArmPos5Hang;
    int cLowerArmPos6Hang;
    int cUpperArmPos6Hang;
    int cDistanceToBackdropForRumble;
    boolean cassetteFull;
    int lastDistanceToBackdrop;

    MotLwrArm = hardwareMap.get(DcMotor.class, "MotLwrArm");
    MotUprArm = hardwareMap.get(DcMotor.class, "MotUprArm");
    SrvPxlPos1 = hardwareMap.get(Servo.class, "SrvPxlPos1");
    SrvPxlPos2 = hardwareMap.get(Servo.class, "SrvPxlPos2");
    SenColPxlPos1 = hardwareMap.get(ColorSensor.class, "SenColPxlPos1");
    SenColPxlPos2 = hardwareMap.get(ColorSensor.class, "SenColPxlPos2");
    MotDrFrLt = hardwareMap.get(DcMotor.class, "MotDrFrLt");
    MotDrBkLt = hardwareMap.get(DcMotor.class, "MotDrBkLt");
    MotDrFrRt = hardwareMap.get(DcMotor.class, "MotDrFrRt");
    MotDrBkRt = hardwareMap.get(DcMotor.class, "MotDrBkRt");
    MotPxlIntk = hardwareMap.get(DcMotor.class, "MotPxlIntk");
    SrvDrLnch = hardwareMap.get(Servo.class, "SrvDrLnch");
    LED4 = hardwareMap.get(LED.class, "LED4");
    LED3 = hardwareMap.get(LED.class, "LED3");
    LED2 = hardwareMap.get(LED.class, "LED2");
    LED1 = hardwareMap.get(LED.class, "LED1");
    SenColPxlPos1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "SenColPxlPos1");
    SenColPxlPos2_DistanceSensor = hardwareMap.get(DistanceSensor.class, "SenColPxlPos2");

    // Initialize Constant Variables
    // Trigger button deadband for any trigger press
    cTrgBtnThresh = 0.1;
    // Controller joystick deadband for the arm motors
    cArmMotorDB = 0.05;
    // Drive motor power level scaling [max 1]
    cDriveBoosted = 1;
    cDriveNormal = 0.9;
    cDriveSlow = 0.5;
    // Arm motor power scaling
    cLowerArmManualMotorPwrScale = 0.6;
    cUpperArmManualMotorPwrScale = 0.5;
    cLowerArmAutoMotorPwr = 0.5;
    cUpperArmAutoMotorPwr = 0.4;
    // Arm position constants
    armPositionState = 0;
    //
    cLowerArmPos0Home = 0;
    cUpperArmPos0Home = 0;
    //
    cLowerArmPos1LiftBox = 0;
    cUpperArmPos1LiftBox = -120;
    //
    cLowerArmPos2LiftArm = -450;
    cUpperArmPos2LiftArm = -120;
    //
    cLowerArmPos3BackScore = -1500;
    cUpperArmPos3BackScore = -460;
    //
    cLowerArmPos4FrontScore = -700;
    cUpperArmPos4FrontScore = 750;
    //
    cLowerArmPos5Hang = -1200;
    cUpperArmPos5Hang = 850;
    //
    cLowerArmPos6Hang = -150;
    cUpperArmPos6Hang = 225;
    cLowerArmPositions = JavaUtil.createListWith(cLowerArmPos0Home, cLowerArmPos1LiftBox, cLowerArmPos2LiftArm, cLowerArmPos3BackScore, cLowerArmPos4FrontScore, cLowerArmPos5Hang, cLowerArmPos6Hang);
    cUpperArmPositions = JavaUtil.createListWith(cUpperArmPos0Home, cUpperArmPos1LiftBox, cUpperArmPos2LiftArm, cUpperArmPos3BackScore, cUpperArmPos4FrontScore, cUpperArmPos5Hang, cUpperArmPos6Hang);
    cArmPositionNames = JavaUtil.createListWith("Home", "Lift Box", "Lift Arm", "Back Score", "Front Score", "Hang", "HangEnd");
    // Servo speeds for the cassette
    cCasFrontToBack = 1;
    cCasBackToFront = 0;
    cCasStop = 0.5;
    cAllowFrontScoreFromCassette = true;
    // Distance to detect pixels in the cassette (cm)
    cPixelPos1Dist = 1;
    cPixelPos2Dist = 10;
    // Distance to detect pixels in the cassette (cm)
    cIntakeIn = -1;
    cIntakeOut = 1;
    cIntakeStop = 0;
    // Rumble when close to the backdrop
    cDistanceToBackdropForRumble = 9;
    cFieldCentric = true;
    // Servo constants for the drone
    cDroneServoStop = 0.5;
    cDroneServoLaunch = 0.1;
    cDroneLaunchDuration = 3000;
    // Initialize Local Variables
    droneLaunchTime = 0;
    cassetteFull = false;
    allowManualArmControl = false;
    triangleButtonPress = 0;
    lastDistanceToBackdrop = 1000;
    // Initialization Routines
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null);
    mImu = new HydraImu_navx(opMode);
    InitArm();
    InitCassette();
    InitDrive();
    InitIntake();
    InitDrone();
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
      ProcessArm();
      ProcessDrive(armPositionState);
      cassetteFull = ProcessCassette(armPositionState);
      ProcessIntake(cassetteFull);
      ProcessDrone();
      // Update telemetry once for all processes
      telemetry.update();
      sleep(20);
    }
    // 12/4/23 - Disabled Distance Sensor  since it is not used
  }

  /**
   * Describe this function...
   */
  private void SetLwrArmPos(int inLwrArmPos) {
    MotLwrArm.setTargetPosition(inLwrArmPos);
    MotLwrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotLwrArm.setPower(cLowerArmAutoMotorPwr);
  }

  /**
   * Describe this function...
   */
  private void SetUprArmPos(int inUprArmPos) {
    MotUprArm.setTargetPosition(inUprArmPos);
    MotUprArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotUprArm.setPower(cUpperArmAutoMotorPwr);
  }

  /**
   * Describe this function...
   */
  private void InitArm() {
    // Brake the motors when power is zero. This keeps the arm from falling
    MotLwrArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    MotUprArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    MotLwrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotUprArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotLwrArm.setTargetPosition(0);
    MotUprArm.setTargetPosition(0);
  }

  /**
   * Describe this function...
   */
  private void InitCassette() {
    // Set servo direction. Position 2 is reversed
    SrvPxlPos1.setDirection(Servo.Direction.FORWARD);
    SrvPxlPos2.setDirection(Servo.Direction.REVERSE);
    // Disable the LEDs since we only need distance measurements
    SenColPxlPos1.enableLed(false);
    SenColPxlPos2.enableLed(false);
    // Ensure the servos are stopped
    SetPixelPos1Dir(cCasStop);
    SetPixelPos2Dir(cCasStop);
  }

  /**
   * Describe this function...
   */
  private void InitDrive() {
    // Set motor directions. Right side motors are reversed
    MotDrFrLt.setDirection(DcMotor.Direction.FORWARD);
    MotDrBkLt.setDirection(DcMotor.Direction.FORWARD);
    MotDrFrRt.setDirection(DcMotor.Direction.REVERSE);
    MotDrBkRt.setDirection(DcMotor.Direction.REVERSE);
  }

  /**
   * Describe this function...
   */
  private void ProcessArm() {
    float upperArmPower;
    int nextArmPosition;
    float lowerArmPower;

    if (gamepad2.triangle) {
      if (triangleButtonPress < 3) {
        triangleButtonPress++;
      }
    } else if (triangleButtonPress == 3) {
      allowManualArmControl = !allowManualArmControl;
      triangleButtonPress = 2;
    } else if (triangleButtonPress > 0) {
      triangleButtonPress--;
    }
    if (allowManualArmControl) {
      armPositionState = 0;
      if (gamepad2.cross) {
        MotUprArm.setPower(0);
        MotLwrArm.setPower(0);
        MotLwrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotUprArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotUprArm.setTargetPosition(0);
        MotLwrArm.setTargetPosition(0);
        allowManualArmControl = false;
      }
      else {
        upperArmPower = gamepad2.right_stick_y;
        lowerArmPower = gamepad2.left_stick_y;
        if (Math.abs(upperArmPower) < cArmMotorDB) {
          upperArmPower = 0;
        }
        if (Math.abs(lowerArmPower) < cArmMotorDB) {
          lowerArmPower = 0;
        }
        if (upperArmPower != 0) {
          MotUprArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          MotUprArm.setPower(cUpperArmManualMotorPwrScale * upperArmPower);
        } else {
          MotUprArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          MotUprArm.setTargetPosition(MotUprArm.getCurrentPosition());
          MotUprArm.setPower(cUpperArmManualMotorPwrScale);
        }
        if (lowerArmPower != 0) {
          MotLwrArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          MotLwrArm.setPower(cLowerArmManualMotorPwrScale * lowerArmPower);
        } else {
          MotLwrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          MotLwrArm.setTargetPosition(MotLwrArm.getCurrentPosition());
          MotLwrArm.setPower(cLowerArmManualMotorPwrScale);
        }
      }
    } else {
      if (AllowArmChange()) {
        nextArmPosition = ArmControlSM();
        SetLwrArmPos(((Integer) JavaUtil.inListGet(cLowerArmPositions, JavaUtil.AtMode.FROM_START, ((nextArmPosition + 1) - 1), false)).intValue());
        SetUprArmPos(((Integer) JavaUtil.inListGet(cUpperArmPositions, JavaUtil.AtMode.FROM_START, ((nextArmPosition + 1) - 1), false)).intValue());
      }
    }
    telemetry.addData("LrArm", MotLwrArm.getCurrentPosition());
    telemetry.addData("UprArm", MotUprArm.getCurrentPosition());
    telemetry.addData("Manual Arm", allowManualArmControl);
    telemetry.addData("ArmPos", JavaUtil.inListGet(cArmPositionNames, JavaUtil.AtMode.FROM_START, ((armPositionState + 1) - 1), false));
  }

  /**
   * Describe this function...
   */
  private void ProcessDrive(int inArmPosition) {
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
      telemetry.addData("Yaw", "disconnected");
    }
    else if (mImu.Calibrating()) {
      telemetry.addData("Yaw", "cal");
    }
    else {
      yaw = mImu.GetYaw();
      telemetry.addData("Yaw", yaw);
    }
    // Get driver controller input
    drive = gamepad1.left_stick_y;
    strafe = -gamepad1.left_stick_x * 1.1;
    if (gamepad1.cross && cFieldCentric) {
      // snap to the nearest 90 deg
      double snapHeading = yaw;
      if (yaw >= -180 && yaw <= -135) {
        snapHeading = -180;
      }
      else if (yaw > -135 && yaw <= -45) {
        snapHeading = -90;
      }
      else if (yaw > -45 && yaw <= 45) {
        snapHeading = 0;
      }
      else if (yaw > 45 && yaw <= 135) {
        snapHeading = 90;
      }
      else if (yaw > 135 && yaw <= 180) {
        snapHeading = 180;
      }
      rotate = -Math.sin((yaw - snapHeading) * Math.PI / 180) * 1.1;
    }
    else {
      rotate = -gamepad1.right_stick_x;
    }
    rotX = strafe * Math.cos(-yaw / 180 * Math.PI) - drive * Math.sin(-yaw / 180 * Math.PI);
    rotY = strafe * Math.sin(-yaw / 180 * Math.PI) + drive * Math.cos(-yaw / 180 * Math.PI);
    if (gamepad1.circle && cFieldCentric) {
      mImu.ResetYaw();
    }
    // Set max drive power based on driver input
    if (gamepad1.left_trigger > cTrgBtnThresh || inArmPosition > 2) {
      // Drive slower for better control
      driveMaxPower = cDriveSlow;
    } else if (gamepad1.right_trigger > cTrgBtnThresh) {
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
    MotDrBkLt.setPower(rearLeftPower * driveMaxPower);
    MotDrBkRt.setPower(rearRightPower * driveMaxPower);
    MotDrFrLt.setPower(frontLeftPower * driveMaxPower);
    MotDrFrRt.setPower(frontRightPower * driveMaxPower);
    telemetry.addData("LeftFront", frontLeftPower);
    telemetry.addData("RightFront", frontRightPower);
    telemetry.addData("LeftRear", rearLeftPower);
    telemetry.addData("RightRear", rearRightPower);
  }

  /**
   * Describe this function...
   */
  private void InitIntake() {
  }

  /**
   * Describe this function...
   */
  private void ProcessIntake(boolean inCassetteFull) {
    float gamepad2RtTrg;
    float gamepad2LtTrg;
    double intakeDir;

    // Check the controller for user input
    gamepad2RtTrg = gamepad2.right_trigger;
    gamepad2LtTrg = gamepad2.left_trigger;
    // Set the desired motor power and direction based on the user input
    if (gamepad2RtTrg > cTrgBtnThresh) {
      // Right trigger sets the motor to bring pixels in
      intakeDir = cIntakeIn * gamepad2RtTrg;
    } else if (gamepad2LtTrg > cTrgBtnThresh) {
      // Left trigger sets the motor to bring pixels out
      intakeDir = cIntakeOut * gamepad2LtTrg;
    } else {
      intakeDir = cIntakeStop;
    }
    // If the user is running the intake, always run out if the cassette is already full
    if (inCassetteFull && (gamepad2RtTrg > cTrgBtnThresh || gamepad2LtTrg > cTrgBtnThresh)) {
      intakeDir = cIntakeOut;
    }
    // Send power to the motor based on the input
    MotPxlIntk.setPower(intakeDir);
    telemetry.addData("intake", intakeDir);
  }

  /**
   * Describe this function...
   */
  private void InitDrone() {
    SrvDrLnch.setDirection(Servo.Direction.FORWARD);
    SrvDrLnch.setPosition(cDroneServoStop);
  }

  /**
   * Describe this function...
   */
  private boolean ProcessCassette(int inArmPosition) {
    boolean pixelInPos1;
    boolean pixelInPos2;
    double pixelPos1ServoPos;
    double pixelPos2ServoPos;

    pixelInPos1 = DetectPixelPos1();
    pixelInPos2 = DetectPixelPos2();
    // Use gamepad Y and B to set the desired direction
    if (gamepad2.right_trigger > cTrgBtnThresh) {
      // User wants to run front to back
      // Don't run servos in certain cases when pixels are loaded
      if (pixelInPos1 && pixelInPos2) {
        pixelPos1ServoPos = cCasStop;
        pixelPos2ServoPos = cCasStop;
      } else if (pixelInPos2) {
        pixelPos1ServoPos = cCasFrontToBack;
        pixelPos2ServoPos = cCasStop;
      } else {
        pixelPos1ServoPos = cCasFrontToBack;
        pixelPos2ServoPos = cCasFrontToBack;
      }
    } else if (gamepad2.left_bumper) {
      // User wants to run back to front
      pixelPos1ServoPos = cCasBackToFront;
      pixelPos2ServoPos = cCasBackToFront;
    } else if (cAllowFrontScoreFromCassette && inArmPosition == 4 && gamepad2.right_bumper) {
      // User wants to score at front of robot
      pixelPos1ServoPos = cCasFrontToBack;
      pixelPos2ServoPos = cCasFrontToBack;
    } else {
      pixelPos1ServoPos = cCasStop;
      pixelPos2ServoPos = cCasStop;
    }
    LED4.enable(pixelInPos2);
    LED3.enable(!pixelInPos2);
    LED2.enable(pixelInPos1);
    LED1.enable(!pixelInPos1);
    SetPixelPos1Dir(pixelPos1ServoPos);
    SetPixelPos2Dir(pixelPos2ServoPos);
    return pixelInPos1 && pixelInPos2;
  }

  /**
   * Describe this function...
   */
  private void ProcessDrone() {
    // Get the current time in milliseconds. The value returned represents
    // the number of milliseconds since midnight, January 1, 1970 UTC.
    if (gamepad1.square && gamepad2.square) {
      SrvDrLnch.setPosition(cDroneServoLaunch);
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      droneLaunchTime = System.currentTimeMillis();
    } else if (droneLaunchTime != 0 && ((System.currentTimeMillis() - droneLaunchTime) > cDroneLaunchDuration)) {
      droneLaunchTime = 0;
      SrvDrLnch.setPosition(cDroneServoStop);
    }
  }

  /**
   * Describe this function...
   */
  private void SetPixelPos1Dir(double inDirection) {
    // Keep Servo position in valid range
    inDirection = Math.min(Math.max(inDirection, 0), 1);
    SrvPxlPos1.setPosition(inDirection);
    telemetry.addData("PixelPos1Servo", inDirection);
  }

  /**
   * Describe this function...
   */
  private void SetPixelPos2Dir(double inDirection) {
    // Keep Servo position in valid range
    inDirection = Math.min(Math.max(inDirection, 0), 1);
    SrvPxlPos2.setPosition(inDirection);
    telemetry.addData("PixelPos2Servo", inDirection);
  }

  /**
   * Describe this function...
   */
  private int ArmControlSM() {
    boolean armMoveToHome;
    boolean armMoveToFront;
    boolean armMoveToBack;
    boolean armMoveToHang;

    armMoveToHome = gamepad2.dpad_down;
    armMoveToFront = gamepad2.dpad_right;
    armMoveToBack = gamepad2.dpad_left;
    armMoveToHang = gamepad2.dpad_up;
    if (armMoveToHome) {
      if (armPositionState == 6) {
        armPositionState = 5;
      } else if (armPositionState > 2) {
        armPositionState = 2;
      } else if (armPositionState != 0) {
        armPositionState += -1;
      }
    } else if (armMoveToBack) {
      if (armPositionState == 6) {
        armPositionState = 5;
      } else if (armPositionState < 2) {
        armPositionState += 1;
      } else {
        armPositionState = 3;
      }
    } else if (armMoveToFront) {
      if (armPositionState == 6) {
        armPositionState = 5;
      } else if (armPositionState < 2) {
        armPositionState += 1;
      } else {
        armPositionState = 4;
      }
    } else if (armMoveToHang) {
      if (armPositionState < 2) {
        armPositionState += 1;
      } else if (armPositionState < 5) {
        armPositionState = 5;
      } else if (armPositionState == 5) {
        armPositionState = 6;
      }
    }
    return armPositionState;
  }

  /**
   * Describe this function...
   */
  private boolean DetectPixelPos1() {
    double distPixelPos1;
    boolean detPixelPos1;

    distPixelPos1 = SenColPxlPos1_DistanceSensor.getDistance(DistanceUnit.CM);
    detPixelPos1 = distPixelPos1 < cPixelPos1Dist;
    telemetry.addData("PixelPos1Dist", distPixelPos1);
    telemetry.addData("Pixel1Detect", detPixelPos1);
    return detPixelPos1;
  }

  /**
   * Describe this function...
   */
  private boolean DetectPixelPos2() {
    double distPixelPos2;
    boolean detPixelPos2;

    distPixelPos2 = SenColPxlPos2_DistanceSensor.getDistance(DistanceUnit.CM);
    detPixelPos2 = distPixelPos2 < cPixelPos2Dist;
    telemetry.addData("PixelPos2Dist", distPixelPos2);
    telemetry.addData("Pixel2Detect", detPixelPos2);
    return detPixelPos2;
  }

  /**
   * Describe this function...
   */
  private boolean AllowArmChange() {
    int upperArmError;
    int lowerArmError;

    upperArmError = Math.abs(MotUprArm.getTargetPosition() - MotUprArm.getCurrentPosition());
    lowerArmError = Math.abs(MotLwrArm.getTargetPosition() - MotLwrArm.getCurrentPosition());
    if (lowerArmError <= 15 && upperArmError <= 15) {
      return true;
    }
    if (!(MotLwrArm.isBusy() || MotUprArm.isBusy())) {
      return true;
    }
    return false;
  }
}
