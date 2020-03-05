package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

public abstract class AutoGeneralD extends SkystoneAuto {

  public void runForCurrentAlliance() {
    driveBase.setPoseEstimate(
        allianceSpecificPoseFromRed(new Pose2d(-33, -63, Math.PI / 2)));

    drive(t -> t.splineTo(allianceSpecificPoseFromRed(new Pose2d(-20, -38, 0)),
        new ConstantInterpolator(allianceSpecificHeadingFromRed(Math.PI / 2)))
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(0, -40))));

    partyUntilItsOver();
  }

}
