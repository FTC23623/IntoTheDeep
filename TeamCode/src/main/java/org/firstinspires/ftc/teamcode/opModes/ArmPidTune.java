package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArm;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Config
@TeleOp(name="ArmPidTune")
public class ArmPidTune extends LinearOpMode {
    public static double LiftPosDeg = 0;
    public static double ExtensionPosInches = 0;
    public static double WristPos = 0.5;
    public static double specArmAngle = 0.0;
    public static ElementTypes elementType = ElementTypes.Sample;
    private ElapsedTime mLoopTime;
    @Override
    public void runOpMode() {
        mLoopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1, gamepad2, elementType);
        Arm arm = new Arm(opMode, false);
        Intake intake = new Intake(opMode);
        SpecimenArm specArm = new SpecimenArm(opMode);
        Claw claw = new Claw(opMode);
        waitForStart();
        mLoopTime.reset();
        while (opModeIsActive()) {
            opMode.mLoopTime = mLoopTime.milliseconds();
            if (arm.Startup(false)) {
                break;
            }
            idle();
            mLoopTime.reset();
        }
        mLoopTime.reset();
        while (opModeIsActive()) {
            opMode.mLoopTime = mLoopTime.milliseconds();
            if (arm.TuneMode()) {
                arm.SetLiftArmAngle(LiftPosDeg);
                arm.SetArmExtension(ExtensionPosInches);
                arm.SetWristPos(WristPos);
                specArm.SetAngle(specArmAngle);
            }
            arm.HandleUserInput();
            arm.Process();
            intake.HandleUserInput();
            claw.HandleUserInput();
            intake.Process();
            specArm.Process();
            telemetry.update();
            mLoopTime.reset();
            idle();
        }
    }
}
