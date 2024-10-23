package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name="ArmPidTune")
public class ArmPidTune extends LinearOpMode {
    public static double LiftPosDeg = 0;
    public static double ExtensionPosInches = 0;
    public static double WristPosDeg = 0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1, gamepad2);
        Arm arm = new Arm(opMode);
        Intake intake = new Intake(opMode);
        waitForStart();
        while (opModeIsActive()) {
            if (!arm.AutoMode()) {
                arm.SetLiftArmAngle(LiftPosDeg);
                arm.SetArmExtension(ExtensionPosInches);
                arm.SetWristAngle(WristPosDeg);
            }
            arm.HandleUserInput();
            arm.Process();
            intake.HandleUserInput();
            intake.Process();
            telemetry.update();
            sleep(20);
        }
    }
}
