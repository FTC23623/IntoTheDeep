package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    @Override
    public void runOpMode() {
        final int sleepTime = 20;
        final ElementTypes elementType = ElementTypes.Sample;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1, gamepad2, elementType, sleepTime);
        Arm arm = new Arm(opMode);
        Intake intake = new Intake(opMode);
        waitForStart();
        while (opModeIsActive()) {
            if (arm.Startup(false)) {
                break;
            }
            sleep(sleepTime);
        }
        while (opModeIsActive()) {
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
            sleep(sleepTime);
        }
    }
}
