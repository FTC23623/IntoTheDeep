package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.types.ElementTypes;

@Config
@TeleOp(name="ArmPidTune")
public class ArmPidTune extends LinearOpMode {
    public static double LiftPosDeg = 0;
    public static double ExtensionPosInches = 0;
    public static double WristPos = 1.0;
    public static ElementTypes elementType = ElementTypes.Sample;
    private ElapsedTime mLoopTime;
    @Override
    public void runOpMode() {
        mLoopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1, gamepad2, elementType);
        Arm arm = new Arm(opMode);
        Intake intake = new Intake(opMode);
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
            }
            arm.HandleUserInput();
            arm.Process();
            intake.HandleUserInput();
            intake.Process();
            telemetry.update();
            mLoopTime.reset();
            idle();
        }
    }
}
