package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public abstract class AutoGeneralB extends SkystoneAuto {
  public void runForCurrentAlliance() {
    driveBase.setPoseEstimate(
        allianceSpecificPoseFromRed(new Pose2d(33, -63, Math.PI / 2)));

    driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-5, -60)))
        .back(6)
        .build());

    partyUntilItsOver();
  }
}
