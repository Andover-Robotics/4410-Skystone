package org.firstinspires.ftc.teamcode.auto;

import android.media.MediaPlayer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.andoverrobotics.core.config.Configuration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.io.IOException;
import java.util.function.Function;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class SkystoneAuto extends LinearOpMode {
  protected Bot bot;
  protected MecanumDriveBase driveBase;
  protected AllianceColor alliance;
  protected Configuration config;
  protected MediaPlayer soundPlayer = new MediaPlayer();

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
    bot.slideSystem.zeroLifts();

    initConfig();
  }

  private void initConfig() {
    try {
      config = Configuration.fromPropertiesFile("auto.properties");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // To prevent literal repetition in runOpMode
  protected void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
    driveBase.followTrajectorySync(trajectory.apply(driveBase.trajectoryBuilder()).build());
  }

  protected void playMusicIfEnabled() {
    switch (config.getString("music")) {
      case "rickRoll":
        soundPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rickroll);
        break;
      case "moscow":
        soundPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.moscau);
        break;
      case "rasputin":
        soundPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rasputin);
        break;
      default:
        soundPlayer = null;
    }

    if (soundPlayer != null) {
      soundPlayer.start();
    }
  }

  protected void partyUntilItsOver() {
    if (config.getBoolean("partyMode")) {
      int[] rainbowColors = new int[3];
      int second;

      playMusicIfEnabled();
      bot.slideSystem.rotateFourBarFullyIn();

      // 20 iterations per second
      while (getRuntime() < 30 && !isStopRequested()) {
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
      cleanup();

    }
  }

  protected void cleanup() {
    if (soundPlayer != null) {
      soundPlayer.stop();
    }
  }

  protected void checkForInterrupt() throws InterruptedException {
    if (Thread.currentThread().isInterrupted() || isStopRequested()) {
      throw new InterruptedException("aborted");
    }
  }
}
