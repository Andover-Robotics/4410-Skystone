package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.function.Function;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class SkystoneAuto extends LinearOpMode {
  protected Bot bot;
  protected MecanumDriveBase driveBase;
  protected AllianceColor alliance;
  protected SoundPlayer soundPlayer = SoundPlayer.getInstance();

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

  // To prevent literal repetition in runOpMode
  protected void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
    driveBase.followTrajectorySync(trajectory.apply(driveBase.trajectoryBuilder()).build());
  }

  protected void rickRoll() {
    soundPlayer.startPlaying(hardwareMap.appContext, R.raw.rickroll);
  }

  protected void partyUntilItsOver() {
    int[] rainbowColors = new int[3];
    int second;

    rickRoll();

    // 20 iterations per second
    while (getRuntime() < 30) {
      sleep(50);

      // set hub colors according to rainbow
      PartyMode.getRainbowColor(getRuntime() * 4, rainbowColors);
      bot.hub1.setLedColor(rainbowColors[0], rainbowColors[1], rainbowColors[2]);
      bot.hub2.setLedColor(rainbowColors[0], rainbowColors[1], rainbowColors[2]);

      second = (int) Math.floor(getRuntime());

      if (second % 2 == 0) {
        bot.slideSystem.openClamp();
        bot.intake.takeOut(0.3);
      } else {
        bot.slideSystem.closeClamp();
        bot.intake.takeIn(0.3);
      }
      idle();
    }

    // clean it up
    soundPlayer.stopPlayingAll();
  }
}
