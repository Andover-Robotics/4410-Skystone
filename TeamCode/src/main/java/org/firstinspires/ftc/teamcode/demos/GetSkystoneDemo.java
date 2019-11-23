package org.firstinspires.ftc.teamcode.demos;

import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.auto.CVSkystoneDetector;
import org.firstinspires.ftc.teamcode.auto.StonePosition;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.function.Function;

public class GetSkystoneDemo extends LinearOpMode {

  CVSkystoneDetector detector;
  private Bot bot;
  private SampleMecanumDriveBase driveBase;

  private boolean alliance; // true for RED, false for BLUE

  private final int TILE_SIZE = 24, SKYSTONE_LENGTH = 8;

  public void runOpMode() {
    runOpMode(true);
  }

  public void runOpMode(boolean isRed) {
    alliance = isRed;
    initFields();
    adjustCvWindowWhileWaitForStart();
    if (alliance) {
      driveBase.setPoseEstimate(new Pose2d(-36, -63, Math.PI / 2));
    } else {
      driveBase.setPoseEstimate(new Pose2d(-36, 63, -Math.PI / 2));
    }

    Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
    telemetry.addData("position", result.first);
    telemetry.addData("confidence", "%.5f", result.second);
    telemetry.update();

    drive(t -> t.forward(TILE_SIZE));
    turn(true, Math.PI / 2);

    drive(t -> t.back(TILE_SIZE));

    switch (result.first) {
      case RIGHT:
        // This code is to remain commented out until we attempt to acquire the SkyStone closer to the SkyBridge
//        drive(t -> t.forward(5 + result.first.offsetLeft * 8));
//        drive(t -> t.forward(0.553));           // See documentation for 11/15/19 for
//        drive(t -> t.strafeLeft(7.73776));      //  where these numbers came from
//        driveBase
//       .turnSync(-0.65);
//        drive(t -> t.back(12));
//        break;
      case CENTER:
      case LEFT:
        drive(t -> t.forward(10 + result.first.offsetLeft * SKYSTONE_LENGTH));
        strafeHorizontally(true, 23);
        break;
      default:
        telemetry.addData("Problem", "result.first wasn't initialized");
        telemetry.update();
        while (!isStopRequested()) ;
        return;
    }


    // intake while moving forward
    bot.intake.takeIn(1);
    drive(t -> t.back(8));
    sleep(300);
    bot.intake.stop();

    // Congrats! You should theoretically have a Skystone.

//    driveToCoordinate(-36, -54, Math.PI / 2);
    strafeHorizontally(false, 23 + 8);
    drive(t -> t.forward(5 + result.first.offsetLeft * SKYSTONE_LENGTH + 2 * TILE_SIZE));

    detector.close();
  }

  private void adjustCvWindowWhileWaitForStart() {
    while (!isStarted()) {
      adjustCvWindow();
      idle();
      sleep(200);
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
    telemetry.update();
  }

  private void initFields() {
    bot = Bot.getInstance(this);
    detector = new CVSkystoneDetector(hardwareMap);
    detector.open();

    driveBase = new SampleMecanumDriveREVOptimized(hardwareMap);
  }

  private void strafeHorizontally(boolean left, double inches) {
    // Note: the boolean left is true if for the RED alliance autonomous, the robot should strafe left
    if (alliance == left) {
      drive(t -> t.strafeLeft(inches));
    } else {
      drive(t -> t.strafeRight(inches));
    }
  }

  private void turn(boolean clockwise, double radians) {
    // Note: the boolean clockwise is true if for the RED alliance autonomous, the robot should turn clockwise
    if (alliance == clockwise) {
      driveBase.turnSync(-1 * radians);

    } else {
      driveBase.turnSync(radians);
    }
  }

  // To prevent literal repetition in runOpMode
  private void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
    driveBase.followTrajectorySync(trajectory.apply(driveBase
        .trajectoryBuilder()).build());
  }
}
