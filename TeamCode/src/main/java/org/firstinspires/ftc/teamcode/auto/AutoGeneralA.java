package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;
import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class AutoGeneralA extends SkystoneAuto {
  private enum CrossingVariant {
    INNER(24 + 12),
    OUTER(24 + 24 + 12);

    public final double absYOffset;

    CrossingVariant(double absYOffset) {
      this.absYOffset = absYOffset;
    }
  }

  private CVSkystoneDetector detector;
  private CrossingVariant deliverCrossVariant = CrossingVariant.INNER,
      parkCrossVariant = CrossingVariant.INNER;

  public void runForColor(AllianceColor alliance) {
    try {
      this.alliance = alliance;
      initFields();
      initCV();

      while (!isStarted()) {
        adjustCvWindow();
        adjustAutoVariants();
        checkForInterrupt();

        idle();
        sleep(100);
        telemetry.update();
      }

      // We will always start with left (phone) facing stones
      driveBase.setPoseEstimate(new Pose2d(
          allianceSpecificPositionFromRed(new Vector2d(-33, -63)),
          alliance == RED ? 0 : Math.PI
      ));

      // Align phone with target
      drive(it -> it.strafeLeft(8));
      checkForInterrupt();

      if (gamepad1.back) return;

      sleep(800);
      Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
      telemetry.addData("position", result.first);
      telemetry.addData("confidence", "%.5f", result.second);
      telemetry.speak("I detected a skystone at " + result.first);
      telemetry.update();
      Log.v("Autonomous A", "position sensed: " + result.first);

      detector.close();

      driveBase.turnSync(Math.PI);

      goToQuarryStone(3 + (alliance == RED ? result.first.offsetLeft : result.first.offsetRight()));
      bot.sideClaw.armDown();
      sleep(600);
      bot.sideClaw.clamp();
      sleep(400);
      bot.sideClaw.armUp();
      checkForInterrupt();

      // Cross Skybridge
      drive(t -> t
          .strafeTo(allianceSpecificPositionFromRed(
              new Vector2d(driveBase.getPoseEstimate().getX(), -deliverCrossVariant.absYOffset)))
          .strafeTo(allianceSpecificPositionFromRed(
              new Vector2d(24, -deliverCrossVariant.absYOffset)))
          .strafeTo(allianceSpecificPositionFromRed(
              new Vector2d(24, -33))));
      checkForInterrupt();

      bot.sideClaw.armDown();
      sleep(600);
      bot.sideClaw.release();
      sleep(400);
      bot.sideClaw.armUp();

      if (config.getBoolean("optionAPullsFoundation")) {
        bot.foundationMover.armDown();
        drive(it -> it.reverse()
            .forward(8)
            .splineTo(allianceSpecificPoseFromRed(new Pose2d(32, -47, Math.PI / 4))));
        drive(it -> it
            .splineTo(allianceSpecificPoseFromRed(new Pose2d(42, -45, 0))));
        bot.foundationMover.armUp();
      }

      // Cross Skybridge again?
      drive(t -> t
          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(driveBase.getPoseEstimate().getX(), -parkCrossVariant.absYOffset)))
          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(0, -parkCrossVariant.absYOffset))));

    } catch (Exception interruption) {
      Log.e("Autonomous A", interruption.toString());
      interruption.printStackTrace();
    } finally {
      cleanup();
    }
  }

  private void pulseIntake(int ms) throws InterruptedException {
    double startTime = getRuntime();
    while ((getRuntime() - startTime) * 1000 < ms && !isStopRequested()) {
      bot.intake.takeIn(config.getDouble("skystoneIntakePower"));
      sleep(700);
      bot.intake.stop();
      sleep(50);
      checkForInterrupt();
    }
  }

  private void adjustAutoVariants() {
    telemetry.addLine("variant control")
        .addData("A", "deliver inner")
        .addData("B", "deliver outer")
        .addData("X", "park inner")
        .addData("Y", "park outer");

    if (gamepad1.a) {       // Cross inside         «DEFAULT»
      deliverCrossVariant = CrossingVariant.INNER;
    }
    if (gamepad1.b) {       // Cross outside
      deliverCrossVariant = CrossingVariant.OUTER;
    }
    if (gamepad1.x) {       // Park inside          «DEFAULT»
      parkCrossVariant = CrossingVariant.INNER;
    }
    if (gamepad1.y) {       // Park outside
      parkCrossVariant = CrossingVariant.OUTER;
    }
    telemetry.addData("Crossing", deliverCrossVariant)
        .addData("Parking", parkCrossVariant);
  }

  //
  private void goToQuarryStone(int nthFromOutermost) {
    drive(t -> t.strafeTo(allianceSpecificPositionFromRed(
        new Vector2d(-24 * 3 + 9 + 8 * nthFromOutermost, 34))));

    driveBase.turnToSync(alliance == RED ? Math.PI : 0);
  }

  private void adjustCvWindow() {
    // Window Controls:
    //      gamepad1 left joystick is x-y movement
    //      Dpad up     - increase box height
    //      Dpad down   - decrease box height
    //      Dpad right  - increase box width
    //      Dpad left   - decrease box width

    if (gamepad1.dpad_up)
      detector.config.stoneHeight++;
    else if (gamepad1.dpad_down)
      detector.config.stoneHeight--;

    if (gamepad1.dpad_right)
      detector.config.stoneWidth++;
    else if (gamepad1.dpad_left)
      detector.config.stoneWidth--;

    detector.config.leftStoneMidX += gamepad1.left_stick_x * 4;
    detector.config.leftStoneMidY -= gamepad1.left_stick_y * 4;

    telemetry.addData("Stone Width", detector.config.stoneWidth);
    telemetry.addData("Stone Height", detector.config.stoneHeight);
    telemetry.addData("Window X", detector.config.leftStoneMidX);
    telemetry.addData("Window Y", detector.config.leftStoneMidY);
  }

  private void initCV() {
    detector = new CVSkystoneDetector(hardwareMap);
    if (alliance == BLUE) {
      detector.config.stoneWidth = 236;
      detector.config.stoneHeight = 131;
      detector.config.leftStoneMidX = 359;
      detector.config.leftStoneMidY = 223;
      // TODO reflect this in config files
    }
    detector.open();
  }


  private void strafeHorizontally(boolean left, double inches) {
    // Note: the boolean left is true if for the RED alliance autonomous, the robot should strafe left
    if ((alliance == RED) == left) {
      drive(t -> t.strafeLeft(inches));
    } else {
      drive(t -> t.strafeRight(inches));
    }
  }

  private void navToOuterSkystoneTrajectory(StonePosition position) {
    double baseSkystoneXOffset = config.getDouble("baseSkystoneXOffset");
    double apparentSkystoneWidth = config.getDouble("apparentSkystoneWidth");

    double xBeforeIntake;
    boolean takeInnerSkystone = (alliance == RED && position == StonePosition.LEFT) ||
        (alliance == BLUE && position == StonePosition.RIGHT);

    if (takeInnerSkystone) {
      xBeforeIntake = -baseSkystoneXOffset + apparentSkystoneWidth;
    } else {
      xBeforeIntake = -baseSkystoneXOffset - apparentSkystoneWidth *
          (alliance == AllianceColor.RED ? position.offsetRight() : position.offsetLeft);
    }

    driveBase.turnSync(allianceSpecificHeadingFromRed(-config.getDouble("intakeAngle")));

    drive(it -> it.strafeTo(allianceSpecificPositionFromRed(
        new Vector2d(xBeforeIntake, -(config.getDouble("quarryY"))))));
  }
}
