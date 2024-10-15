package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Config
@TeleOp(name="ArmPidTune")
public class ArmPidTune extends LinearOpMode {
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;
    public static int position = 0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null, null, null);
        Arm arm = new Arm(opMode);
        waitForStart();
        while (opModeIsActive()) {
            arm.Test(p, i, d, f, position);
            arm.Process();
            telemetry.update();
            sleep(20);
        }
    }
}
