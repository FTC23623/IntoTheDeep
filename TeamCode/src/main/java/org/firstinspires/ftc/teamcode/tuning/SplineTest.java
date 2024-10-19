package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(30, 63, HeadingRad(-90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                        .splineToLinearHeading(new Pose2d(59,46,HeadingRad(-90)),HeadingRad(-90))
                        .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                        .splineToLinearHeading(new Pose2d(49,46, HeadingRad(-90)),HeadingRad(-90))
                        .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                        .splineToLinearHeading(new Pose2d(51,26,HeadingRad(0)),HeadingRad(90))
                        .splineToLinearHeading(new Pose2d(54,54, HeadingRad(-135)),HeadingRad(0))
                        // .lineTo(new Pose2d(56,15, HeadingRad(180)), HeadingRad(180))
                        // .splineToLinearHeading(new Pose2d(38,10,HeadingRad(180)),HeadingRad(-90))
                        .splineToLinearHeading(new Pose2d(46,54, HeadingRad(-90)), HeadingRad(0))
                        .splineToLinearHeading(new Pose2d(25,10,HeadingRad(180)),HeadingRad(180))
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }

    public static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }
}
