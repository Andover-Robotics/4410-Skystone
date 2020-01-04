package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;
import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import kotlin.Unit;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class AutoGeneralA extends SkystoneAuto {

  private CVSkystoneDetector detector;

  public void runForColor(AllianceColor alliance) {
    try {
      this.alliance = alliance;
      initFields();
      initCV();

      adjustCvWindowWhileWaitForStart();
      checkForInterrupt();

      driveBase.setPoseEstimate(allianceSpecificPoseFromRed(new Pose2d(-33, -63, Math.PI / 2)));

      // Align phone with target
      drive(it -> it.forward(8));
      driveBase.turnSync(-Math.PI / 2);
      checkForInterrupt();

      if (gamepad1.back) return;

      sleep(800);
      Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
      telemetry.addData("position", result.first);
      telemetry.addData("confidence", "%.5f", result.second);
      telemetry.update();
      Log.v("Autonomous A", "position sensed: " + result.first);

      detector.close();

      if (alliance == BLUE) {
        driveBase.turnSync(Math.PI);
      }

      navToOuterSkystoneTrajectory(result.first);
      checkForInterrupt();

      bot.slideSystem.prepareToIntake();
      sleep(100);
      while (bot.slideSystem.isLiftRunningToPosition() && !isStopRequested()) ;
      checkForInterrupt();

      // intake while moving forward
      driveBase.setDrivePower(new Pose2d(-(config.getDouble("intakeSpeed")), 0, 0));
      pulseIntake(2500);
      driveBase.setDrivePower(new Pose2d(0, 0, 0));
      checkForInterrupt();

      bot.slideSystem.startRunningLiftsToBottom();
      strafeHorizontally(false, 20);
      checkForInterrupt();

      // Prepare to cross the Skybridge
      while (bot.slideSystem.isLiftRunningToPosition() && !isStopRequested()) ;
      if (isStopRequested()) return;

      bot.slideSystem.relaxLift();
      bot.slideSystem.closeClamp();
      checkForInterrupt();

      drive(t -> t.strafeTo(allianceSpecificPositionFromRed(new Vector2d(20, -40)))
          .addMarker(() -> {
            bot.slideSystem.setLiftTargetPosition(950);
            bot.slideSystem.runLiftsToTargetPosition(1.1);
            return Unit.INSTANCE;
          })
          .splineTo(allianceSpecificPoseFromRed(new Pose2d(34, -30, Math.PI / 2)),
              new LinearInterpolator(driveBase.getExternalHeading(), allianceSpecificHeadingFromRed(Math.PI / 2)))
          .forward(3));
      checkForInterrupt();

      while (bot.slideSystem.isLiftRunningToPosition() && !isStopRequested()) ;

      bot.intake.takeOut(0.6);
      bot.slideSystem.rotateFourBarToRelease();
      sleep(1300);
      bot.slideSystem.releaseClamp();
      sleep(400);

      bot.slideSystem.rotateFourBarToTop();
      sleep(1300);
      bot.slideSystem.openClamp();

      bot.slideSystem.startRunningLiftsToBottom();
      bot.intake.stop();

      if (config.getBoolean("optionAPullsFoundation")) {
        bot.foundationMover.armDown();
        drive(it -> it.reverse()
            .forward(8)
            .splineTo(allianceSpecificPoseFromRed(new Pose2d(32, -47, Math.PI / 4))));
        drive(it -> it
            .splineTo(allianceSpecificPoseFromRed(new Pose2d(42, -45, 0))));
        bot.foundationMover.armUp();
      }

      while (bot.slideSystem.isLiftRunningToPosition() && !isStopRequested()) ;

      drive(t -> t
          .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-10, -38))));

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

  private void adjustCvWindowWhileWaitForStart() {
    while (!isStarted()) {
      adjustCvWindow();
      idle();

      sleep(200);
      telemetry.update();
    }
  }

  private void adjustCvWindow() {
    // Window Controls:
    //      gamempad1 left joystick is x-y movement
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
