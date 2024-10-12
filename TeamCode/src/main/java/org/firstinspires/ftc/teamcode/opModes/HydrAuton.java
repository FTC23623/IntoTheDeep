package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

abstract public class HydrAuton extends LinearOpMode {
    protected SampleMecanumDrive mDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize
        mDrive = new SampleMecanumDrive(hardwareMap);
        // wait for the opmode to start
        waitForStart();
        // run the opmode
        while (opModeIsActive()) {
            AutonSm();
        }
    }

    abstract void AutonSm();
}
