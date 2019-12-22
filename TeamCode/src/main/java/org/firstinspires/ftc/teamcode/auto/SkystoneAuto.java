package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class SkystoneAuto extends LinearOpMode {
  protected Bot bot;
  protected MecanumDriveBase driveBase;
  protected AllianceColor alliance;

  protected Pose2d allianceSpecificPoseFromRed(Pose2d redPose) {
    return new Pose2d(
        allianceSpecificPositionFromRed(redPose.vec()),
        allianceSpecificHeadingFromRed(redPose.getHeading()));
  }
  protected Vector2d allianceSpecificPositionFromRed(Vector2d redPos) {
    return alliance == RED ? redPos : new Vector2d(redPos.getX(), -redPos.getY());
  }
  protected double allianceSpecificHeadingFromRed(double redHeading) {
    return alliance == RED ? redHeading : -redHeading;
  }

  protected void initFields() {
    bot = Bot.getInstance(this);
    driveBase = new MecanumDriveREVOptimized(hardwareMap);
  }
}
