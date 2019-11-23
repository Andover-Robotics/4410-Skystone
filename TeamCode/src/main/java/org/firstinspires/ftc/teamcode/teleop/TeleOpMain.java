package org.firstinspires.ftc.teamcode.teleop;

import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.hardware.Jack;

@TeleOp(name = "4410-2020 TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
  private Bot bot;
  private Jack jack;

  @Override
  public void init() {
    bot = Bot.getInstance(this);

    // Temporary addition for scrimmage
    jack = new Jack(hardwareMap.dcMotor.get("jackLeft"), hardwareMap.dcMotor.get("jackRight"));
  }

  @Override
  public void loop() {
    long startMillis = System.currentTimeMillis();

    driveFieldCentric();
    controlFoundationMovers();
    controlIntake();
    controlJack();

    telemetry.addData("cycle period", "%d ms", System.currentTimeMillis() - startMillis);
  }

  private void controlJack() {
    if (gamepad1.dpad_down) {
      jack.setPower(0.6);
    } else if (gamepad1.dpad_up) {
      jack.setPower(-0.6);
    } else {
      jack.setPower(0);
    }
  }

  private void controlIntake() {
    double intakeSpeed = gamepad1.left_trigger / 2;
    double outtakeSpeed = gamepad1.right_trigger * 0.4;

    if (intakeSpeed > 0) {
      bot.intake.takeIn(intakeSpeed);
    } else if (outtakeSpeed > 0) {
      bot.intake.takeOut(outtakeSpeed);
    } else {
      bot.intake.stop();
    }
  }

  private void controlFoundationMovers() {
    if (gamepad1.left_bumper || gamepad1.right_bumper) {
      bot.foundationMover.armDown();
    } else {
      bot.foundationMover.armUp();
    }
  }

  private void driveFieldCentric() {
    Coordinate driveVector = Coordinate.fromXY(gamepad1.left_stick_x, -gamepad1.left_stick_y)
        .rotate((int) -bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      bot.driveTrain.setRotationPower(gamepad1.right_stick_x);
    }
    bot.driveTrain.setStrafeRotation(driveVector, driveVector.getPolarDistance(), gamepad1.right_stick_x);
  }
}
