package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;
import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class AutoGeneralA extends SkystoneAuto {
  private enum CrossingVariant {
    INNER(24 + 12 + 2),
    OUTER(24 + 24 + 12);

    public final double absYOffset;

    CrossingVariant(double absYOffset) {
      this.absYOffset = absYOffset;
    }
  }

  private CVSkystoneDetector detector;
  private CrossingVariant deliverCrossVariant = CrossingVariant.INNER,
      parkCrossVariant = CrossingVariant.INNER;
  private boolean getSecondSkystoneIfPossible = true;
  private boolean crossSkybridgeWithSplines = false;

  public void runForColor(AllianceColor alliance) {
    try {
      this.alliance = alliance;
      initFields();
      initCV();
      bot.foundationMover.armDown();

      while (!isStarted()) {
        adjustCvWindow();
        adjustAutoVariants();
        checkForInterrupt();

        idle();
        sleep(100);
        telemetry.update();
      }

      // We will always start with left (phone) facing stones
      if (alliance == RED) {
        driveBase.setPoseEstimate(new Pose2d(-33, -63, 0));
      } else {
        // To facilitate different starting positions
        driveBase.setPoseEstimate(new Pose2d(-30.8, 63, Math.PI));
      }
      checkForInterrupt();

      if (gamepad1.back) return;

      bot.foundationMover.armUp();
      Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
      telemetry.addData("position", result.first);
      telemetry.addData("confidence", "%.5f", result.second);
      telemetry.speak("I detected a skystone at " + result.first);
      telemetry.update();
      Log.v("Autonomous A", "position sensed: " + result.first);

      detector.close();

      int skystoneOffset = alliance == RED ? result.first.offsetLeft : result.first.offsetRight();

      deliverStone(null, 3 + skystoneOffset, 1);

      boolean secondSkystoneNextToFieldWall = (alliance == RED && result.first == StonePosition.LEFT) ||
          (alliance == BLUE && result.first == StonePosition.RIGHT);
      if (getSecondSkystoneIfPossible && !secondSkystoneNextToFieldWall) {
        deliverStone(driveBase.trajectoryBuilder()
            .strafeTo(allianceSpecificPositionFromRed(
                new Vector2d(0, -(deliverCrossVariant.absYOffset + 3)))), skystoneOffset, 2);
      }

      repositionFoundation();

      // Cross Skybridge again?
//      drive(t -> t
//          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(driveBase.getPoseEstimate().getX(), -parkCrossVariant.absYOffset)))
//          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(0, -parkCrossVariant.absYOffset))));
      drive(t -> t.back(30));

    } catch (Exception interruption) {
      Log.e("Autonomous A", interruption.toString());
      interruption.printStackTrace();
    } finally {
      cleanup();
    }
  }

  private void deliverStone(BaseTrajectoryBuilder trajectoryBeforeGo, int nthStoneFromWall, int nth) throws InterruptedException {
    bot.sideClaw.armDisabled();
    bot.sideClaw.clamp();
    goToQuarryStoneAndLowerSideClaw(trajectoryBeforeGo, nthStoneFromWall);
    driveBase.setDrivePower(new Pose2d(0, -0.19));
    sleep(450);
    bot.sideClaw.clamp();
    sleep(450);
    driveBase.setDrivePower(new Pose2d(0, 0));
    bot.sideClaw.armDisabled();
    double heading = alliance == RED ? Math.PI : 0;
//    driveBase.turnToSync(heading);
    checkForInterrupt();
    int depositX = 25 + nth * 8;

    // Cross Skybridge
    if (crossSkybridgeWithSplines) {
      drive(t -> t
          .setReversed(alliance == RED)
          .splineTo(allianceSpecificPoseFromRed(
              new Pose2d(-5, -(deliverCrossVariant.absYOffset + 3), heading)))
          .lineTo(allianceSpecificPositionFromRed(
              new Vector2d(5, -(deliverCrossVariant.absYOffset + 3))))
          .splineTo(allianceSpecificPoseFromRed(
              new Pose2d(depositX, -36, heading))));


    } else {
      drive(t -> t
          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-5, -(deliverCrossVariant.absYOffset + 2))))
          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(5, -(deliverCrossVariant.absYOffset + 2))))
          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(depositX, -33))));
    }
    checkForInterrupt();

    bot.sideClaw.armRelease();
    bot.sideClaw.release();
    sleep(300);
    bot.sideClaw.armDisabled();
    checkForInterrupt();
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
        .addData("Y", "park outer")
        .addData("Right bumper", "get second Skystone if possible")
        .addData("Left bumper", "use splines");

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
    if (gamepad1.right_bumper) {
      getSecondSkystoneIfPossible = true;
    }
    if (gamepad1.left_bumper) {
      crossSkybridgeWithSplines = true;
    }
    telemetry.addData("Crossing", deliverCrossVariant)
        .addData("Parking", parkCrossVariant)
        .addData("2nd Skystone", getSecondSkystoneIfPossible ? "yes" : "no")
        .addData("splines", crossSkybridgeWithSplines ? "enabled" : "disabled");
  }

  private void goToQuarryStoneAndLowerSideClaw(BaseTrajectoryBuilder prevTrajectory, int nthFromOutermost) {
    double targetHeading = alliance == RED ? Math.PI : (driveBase.getExternalHeading() < Math.PI ? 0 : 2 * Math.PI);
      Vector2d targetPos = allianceSpecificPositionFromRed(new Vector2d(-24 * 3 + 4.5 + 8 * nthFromOutermost, -34.9));
    drive(t -> (prevTrajectory == null ? t : prevTrajectory).lineTo(targetPos,
        new LinearInterpolator(driveBase.getExternalHeading(), targetHeading - driveBase.getExternalHeading())));
    bot.sideClaw.release();
    sleep(300);
    bot.sideClaw.armDown();
//    drive(t -> t.strafeTo(targetPos));
    driveBase.turnToSync(targetHeading);
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
      detector.config.stoneWidth = 207;
      detector.config.stoneHeight = 101;
      detector.config.leftStoneMidX = 390;
      detector.config.leftStoneMidY = 401;
      // TODO reflect this in config files
    }
    detector.open();
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

  private void repositionFoundation() {
    bot.foundationMover.armUp();
    driveBase.turnToSync(allianceSpecificHeadingFromRed(Math.PI / 2));
    drive(it -> it.strafeTo(allianceSpecificPositionFromRed(new Vector2d(41, -24))));
    driveBase.setDrivePower(new Pose2d(0.1, 0, 0.02));
    sleep(150);
    driveBase.setDrivePower(new Pose2d(0.1, 0, 0.06));
    bot.foundationMover.armDown();
    sleep(500);

    driveBase.followTrajectorySync(new TrajectoryBuilder(driveBase.getPoseEstimate(), new DriveConstraints(
        50, 20, 0, Math.PI/3, Math.PI/6, 0))
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(36, -36)))
        .lineTo(allianceSpecificPositionFromRed(new Vector2d(22, -41)),
            new LinearInterpolator(allianceSpecificHeadingFromRed(Math.PI / 2),
                -allianceSpecificHeadingFromRed(Math.PI / 2)))
        .build());

    bot.foundationMover.armUp();
    bot.sideClaw.clamp();
    driveBase.setDrivePower(new Pose2d(0.27, 0));
    sleep(800);
  }
}
