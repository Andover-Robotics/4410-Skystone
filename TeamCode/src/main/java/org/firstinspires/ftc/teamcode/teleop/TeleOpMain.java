package org.firstinspires.ftc.teamcode.teleop;

import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(name = "4410-2020 TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
  private Bot bot;
  public static double driveSpeed = 1;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    bot.slideSystem.zeroLifts();
  }

  @Override
  public void loop() {
    long startMillis = System.currentTimeMillis();

    driveFieldCentric();
    controlFoundationMovers();
    controlIntake();

    ControlState.runLoop(this);
    ControlState.updateStage(this);

    telemetry.addData("cycle period", "%d ms", System.currentTimeMillis() - startMillis);
    telemetry.addData("current control state", ControlState.currentStage);
    telemetry.addData("current draw 1", bot.hub1.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
    telemetry.addData("current draw 2", bot.hub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));


    if (gamepad1.right_stick_button) {
      // Reset field centric (set current heading to human heading)
      bot.initImu(this);
    }
  }

  private void controlIntake() {
    double intakeSpeed = gamepad1.left_trigger * 0.5;
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

    double microMultiplier = gamepad1.left_stick_button ? 0.2 : driveSpeed;

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      bot.driveTrain.setRotationPower(gamepad1.right_stick_x);
    } else {
      bot.driveTrain.setStrafeRotation(driveVector,
          driveVector.getPolarDistance() * microMultiplier, gamepad1.right_stick_x);
    }
  }
}
