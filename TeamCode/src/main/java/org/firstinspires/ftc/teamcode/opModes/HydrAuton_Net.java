package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.types.NetAutonStates;

import java.util.Map;

public class HydrAuton_Net extends HydrAuton {
    private NetAutonStates mState;
    Map<NetAutonStates, Trajectory> mTrajectories;
    private final Pose2d mStartPose = new Pose2d(0, 0, 0);

    public void InitializeTrajectories() {
        mTrajectories.put(NetAutonStates.NetAuton_DrivingToSample1, mDrive.trajectoryBuilder(mStartPose)
                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                .build());;
        mTrajectories.put(NetAutonStates.state1, mDrive.trajectoryBuilder(mTrajectories.get(NetAutonStates.NetAuton_DrivingToSample1).end())
                .splineTo(new Vector2d(45, 0), Math.toRadians(90))
                .build());
    }

    @Override
    public void AutonSm() {
        switch (mState) {
            case NetAuton_Start:
                mState = NetAutonStates.NetAuton_DrivingToSample1;
                mDrive.followTrajectoryAsync(mTrajectories.get(mState));
                break;
            case NetAuton_DrivingToSample1:
                // wait until drive is complete
                if (!mDrive.isBusy()) {
                    mState = NetAutonStates.NetAuton_PickUpSample;
                }
                break;
            case state1:
                break;

        }
    }
}
