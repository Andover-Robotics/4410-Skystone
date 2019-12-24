package org.firstinspires.ftc.teamcode.auto;

import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.andoverrobotics.core.config.Configuration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.io.IOException;
import java.util.function.Function;

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

    // Align phone with target
    drive(it -> it.forward(8));
    driveBase.turnSync(-Math.PI / 2);

    Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
    telemetry.addData("position", result.first);
    telemetry.addData("confidence", "%.5f", result.second);
    telemetry.update();

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

    driveBase.followTrajectorySync(navToOuterSkystoneTrajectory(result.first));

    // intake while moving forward
    bot.intake.takeIn(config.getDouble("skystoneIntakePower"));
    drive(t -> t.back(7));
    sleep(500);
    bot.intake.stop();

    // Congrats! You should theoretically have a Skystone.
    strafeHorizontally(false, 22);

    drive(t -> t.strafeTo(allianceSpecificPositionFromRed(new Vector2d(20, -40))));
    driveBase.turnSync(Math.PI);

    bot.intake.takeOut(0.7);
    sleep(600);
    bot.intake.stop();

    drive(t -> t
        .splineTo(allianceSpecificPoseFromRed(new Pose2d(20, -40, Math.PI/2)))
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-5, -42))));
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

  private Trajectory navToOuterSkystoneTrajectory(StonePosition position) {
    double baseSkystoneXOffset = config.getDouble("baseSkystoneXOffset");
    double apparentSkystoneWidth = config.getDouble("apparentSkystoneWidth");

    double xBeforeIntake = -baseSkystoneXOffset - apparentSkystoneWidth *
            (alliance == AllianceColor.RED ? position.offsetRight() : position.offsetLeft);


    telemetry.addData("xBeforeIntake", xBeforeIntake);
    telemetry.update();

    return driveBase.trajectoryBuilder()
        .splineTo(allianceSpecificPoseFromRed(new Pose2d(xBeforeIntake, -38, 0.5)))
        .strafeTo(allianceSpecificPositionFromRed(new Vector2d(xBeforeIntake, -(config.getDouble("quarryY")))))
        .build();
  }
}
