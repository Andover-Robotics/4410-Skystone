package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.FoundationMover;
import org.firstinspires.ftc.teamcode.hardware.Intake;

@TeleOp(name = "4410-2020 TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
  private SampleMecanumDriveBase drive;
  private FoundationMover foundationMover;
  private Intake intake;
  private Bot bot;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
  }

  @Override
  public void loop() {
    long startMillis = System.currentTimeMillis();

    driveFieldCentric();
    controlFoundationMovers();
    controlIntake();

    telemetry.addData("cycle period", "%d ms", System.currentTimeMillis() - startMillis);
  }

  private void controlIntake() {
    double intakeSpeed = gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger / 2 : 0;
    double outtakeSpeed = gamepad1.right_trigger > 0.1 ? gamepad1.right_trigger / 2 : 0;

    if (intakeSpeed > 0) {
      intake.takeIn(intakeSpeed);
    } else if (outtakeSpeed > 0) {
      intake.takeOut(outtakeSpeed);
    } else {
      intake.stop();
    }
  }

  private void controlFoundationMovers() {
    if (gamepad1.left_bumper || gamepad1.right_bumper) {
      foundationMover.armDown();
    } else {
      foundationMover.armUp();
    }
  }

  private void driveFieldCentric() {
    Coordinate driveVector = Coordinate.fromXY(gamepad1.left_stick_x, -gamepad1.left_stick_y)
        .rotate((int) -bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

    drive.setDrivePower(new Pose2d(
        driveVector.getY(),
        -driveVector.getX(),
        -gamepad1.right_stick_x
    ));
    drive.update();
  }
}
