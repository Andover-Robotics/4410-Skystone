package org.firstinspires.ftc.teamcode.demos;

import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.auto.CVSkystoneDetector;
import org.firstinspires.ftc.teamcode.auto.StonePosition;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Intake;

import java.util.function.Function;

@Autonomous(name = "Get Skystone", group = "Autonomous")
public class GetSkystoneDemo extends LinearOpMode {

  CVSkystoneDetector detector;
  private SampleMecanumDriveBase driveBase;
  private Intake intake;

  public void runOpMode() {
    initFields();
    adjustCvWindowWhileWaitForStart();

    Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
    telemetry.addData("position", result.first);
    telemetry.addData("confidence", "%.5f", result.second);
    telemetry.update();

    drive(t -> t.forward(24));
    driveBase.turnSync(-1 * (Math.PI / 2));

    switch (result.first) {
      case RIGHT:
        drive(t -> t.forward(0.553));           // See documentation for 11/15/19 for
        drive(t -> t.strafeLeft(7.73776));      //  where these numbers came from
        driveBase.turnSync(-0.65);
        drive(t -> t.back(12));
        break;
      case CENTER:
      case LEFT:
        drive(t -> t.forward(5 + result.first.offsetLeft * 8));
        drive(t -> t.strafeLeft(17));
        break;
      default:
        telemetry.addData("Problem", "result.first wasn't initialized");
        telemetry.update();
        while (!isStopRequested()) ;
        return;
    }


    // intake while moving forward
    intake.takeIn(1);
    drive(t -> t.back(2));
    sleep(300);
    intake.stop();

    // Congrats! You should theoretically have a Skystone.

    driveToCoordinate(-36, -54, Math.PI / 2);

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

  ;

  private void initFields() {
    driveBase = new SampleMecanumDriveREVOptimized(hardwareMap);

    intake = new Intake(hardwareMap.dcMotor.get("intakeLeft"), hardwareMap.dcMotor.get("intakeLeft"));

    detector = new CVSkystoneDetector(hardwareMap);
    detector.open();
  }

  // To prevent literal repetition in runOpMode
  private void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
    driveBase.followTrajectorySync(trajectory.apply(driveBase.trajectoryBuilder()).build());
  }

  // To return to a specified point
  private void driveToCoordinate(double x, double y, double heading) {
    driveBase.followTrajectorySync(
        driveBase.trajectoryBuilder().reverse()
            .splineTo(new Pose2d(x, y, heading)).build()
    );
  }

}
