package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

public abstract class AutoGeneralD extends SkystoneAuto {

  public void runForCurrentAlliance() {
    driveBase.setPoseEstimate(
        allianceSpecificPoseFromRed(new Pose2d(-33, -63, Math.PI / 2)));

    // Moves forward
    driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-33, -35)))
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(0, -40)))
        .build());

    partyUntilItsOver();
  }

}
