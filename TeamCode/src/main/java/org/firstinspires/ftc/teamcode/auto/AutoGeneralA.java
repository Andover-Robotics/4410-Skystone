package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;
import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.andoverrobotics.core.config.Configuration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import kotlin.Unit;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.io.IOException;
import java.util.function.Function;

import static org.firstinspires.ftc.teamcode.util.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.util.AllianceColor.RED;

public abstract class AutoGeneralA extends SkystoneAuto {

  private CVSkystoneDetector detector;
  private Configuration config;

  public void runForColor(AllianceColor alliance) {
    this.alliance = alliance;
    initFields();
    initCV();
    initConfig();

    adjustCvWindowWhileWaitForStart();
    if (isStopRequested()) return;

    driveBase.setPoseEstimate(allianceSpecificPoseFromRed(new Pose2d(-33, -63, Math.PI / 2)));
    rickRoll();

    // Align phone with target
    drive(it -> it.forward(8));
    driveBase.turnSync(-Math.PI / 2);

    if (gamepad1.back) return;

    Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
    telemetry.addData("position", result.first);
    telemetry.addData("confidence", "%.5f", result.second);
    telemetry.update();
    Log.v("Autonomous A", "position sensed: " + result.first);

    detector.close();

//    switch (result.first) {
//      case RIGHT:
//        // This code is to remain commented out until we attempt to acquire the SkyStone closer to the SkyBridge
////        drive(t -> t.forward(5 + result.first.offsetLeft * 8));
////        drive(t -> t.forward(0.553));           // See documentation for 11/15/19 for
////        drive(t -> t.strafeLeft(7.73776));      //  where these numbers came from
////        driveBase
////       .turnSync(-0.65);
////        drive(t -> t.back(12));
////        break;
//      case CENTER:
//      case LEFT:
//        drive(t -> t.forward(9 + result.first.offsetLeft * SKYSTONE_LENGTH));
//        strafeHorizontally(true, 20);
//        break;
//      default:
//        telemetry.addData("Problem", "result.first wasn't initialized");
//        telemetry.update();
//        while (!isStopRequested()) ;
//        return;
//    }
    if (alliance == BLUE) {
      driveBase.turnSync(-Math.PI);
    }

    navToOuterSkystoneTrajectory(result.first);

    bot.slideSystem.prepareToIntake();
    sleep(100);
    while (bot.slideSystem.isLiftBusy() && !isStopRequested());
    if (isStopRequested()) return;

    // intake while moving forward
    driveBase.followTrajectory(driveBase.trajectoryBuilder().back(9).build());
    pulseIntake(1500);
    driveBase.waitForIdle();

    bot.slideSystem.startRunningLiftsToBottom();
    // Congrats! You should theoretically have a Skystone.
    strafeHorizontally(false, 17);

    // Prepare to cross the Skybridge
    while (bot.slideSystem.isLiftBusy() && !isStopRequested());
    if (isStopRequested()) return;

    bot.slideSystem.relaxLift();
    bot.slideSystem.closeClamp();

    drive(t -> t.strafeTo(allianceSpecificPositionFromRed(new Vector2d(20, -35)))
        .addMarker(() -> {
          bot.slideSystem.setLiftTargetPosition(1200);
          bot.slideSystem.runLiftsToTargetPosition(1.1);
          return Unit.INSTANCE;
        })
        .splineTo(allianceSpecificPoseFromRed(new Pose2d(40, -24, Math.PI / 2.2)))
        .forward(3));
    if (isStopRequested()) return;

    while (bot.slideSystem.isLiftBusy() && !isStopRequested());

    bot.slideSystem.rotateFourBarToRelease();
    sleep(1300);
    bot.slideSystem.releaseClamp();
    sleep(400);

    bot.slideSystem.rotateFourBarToTop();
    sleep(1300);
    bot.slideSystem.openClamp();

    bot.slideSystem.startRunningLiftsToBottom();

    if (config.getBoolean("optionAPullsFoundation")) {
      bot.foundationMover.armDown();
      drive(it -> it.reverse()
          .splineTo(allianceSpecificPoseFromRed(new Pose2d(36, -48, 0))));
      bot.foundationMover.armUp();
      drive(it -> it.forward(14));
    }

    while (bot.slideSystem.isLiftBusy() && !isStopRequested());

    drive(t -> t
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-12, -37))));
  }

  private void pulseIntake(int ms) {
    double startTime = getRuntime();
    while ((getRuntime() - startTime) * 1000 < ms && !isStopRequested()) {
      bot.intake.takeIn(config.getDouble("skystoneIntakePower"));
      sleep(600);
      bot.intake.stop();
      sleep(200);
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

  private void initConfig() {
    try {
      config = Configuration.fromPropertiesFile("auto.properties");
    } catch (IOException e) {
      e.printStackTrace();
    }
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

    telemetry.addData("xBeforeIntake", xBeforeIntake);
    telemetry.update();

    driveBase.turnSync(allianceSpecificHeadingFromRed(-config.getDouble("intakeAngle")));

    drive(it -> it.strafeTo(allianceSpecificPositionFromRed(
        new Vector2d(xBeforeIntake, -(config.getDouble("quarryY"))))));
  }
}
