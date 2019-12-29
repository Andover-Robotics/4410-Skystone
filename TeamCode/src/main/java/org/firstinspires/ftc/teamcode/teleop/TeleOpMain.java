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
  private boolean useSimpleAutomation = false;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    bot.slideSystem.zeroLifts();
  }

  public void init_loop() {
    if (gamepad1.x) {
      useSimpleAutomation = true;
      telemetry.addData("automation is now", "simple");
      telemetry.update();
    }
  }

  @Override
  public void loop() {
    long startMillis = System.currentTimeMillis();

    driveFieldCentric();
    controlFoundationMovers();
    controlIntake();

    if (useSimpleAutomation) {
      automateSimply();
    } else {
      ControlState.runLoop(this);
      ControlState.updateStage(this);
    }

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
        .add(getMicroAdjustmentStrafeCoordinate().scale(0.3))
        .rotate((int) -bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

    double microMultiplier = gamepad1.left_stick_button ? 0.2 : driveSpeed;
    double rotation = gamepad1.right_stick_x + getMicroAdjustmentRotation() * 0.2;

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      bot.driveTrain.setRotationPower(rotation);
    } else {
      bot.driveTrain.setStrafeRotation(driveVector,
          driveVector.getPolarDistance() * microMultiplier, rotation);
    }
  }

  private Coordinate getMicroAdjustmentStrafeCoordinate() {
    int x = 0, y = 0;
    if (gamepad1.dpad_up) y += 1;
    if (gamepad1.dpad_down) y -= 1;
    if (gamepad1.dpad_right) x += 1;
    if (gamepad1.dpad_left) x -= 1;
    return Coordinate.fromXY(x, y);
  }

  private int getMicroAdjustmentRotation() {
    return (gamepad1.b ? 1 : 0) + (gamepad1.x ? -1 : 0);
  }

  private void automateSimply() {

    // left y: lift
    if (Math.abs(gamepad2.left_stick_y) > 0.1) {
      bot.slideSystem.setLiftPower(-gamepad2.left_stick_y);
    } else {
      bot.slideSystem.holdLiftHeight();
    }
    if (gamepad2.right_bumper) {
      bot.slideSystem.relaxLift();
    } else if (gamepad2.left_bumper) {
      bot.slideSystem.startAligningLiftSets();
    }

    // right y: clamp
    if (gamepad2.right_stick_y < -0.2) {
      bot.slideSystem.setClampSpeed(gamepad2.right_stick_y);
    }
    else if (gamepad2.right_stick_y > 0.4) {
      bot.slideSystem.closeClamp();
    }
    else if (gamepad2.back) {
      bot.slideSystem.releaseClamp();
    }

    // right x: four bar
    bot.slideSystem.setFourBarSpeed(gamepad2.right_stick_x);

    if (gamepad2.a) {
      // Pickup
      bot.slideSystem.prepareToIntake();
    }
    if (gamepad2.x) {
      // Delivery
      bot.slideSystem.startRunningLiftsToBottom();
    }
    if (gamepad2.y) {
      // Stacking
      bot.slideSystem.rotateFourBarToRelease();
    }
    if (gamepad2.b) {
      // Return
      bot.slideSystem.startRunningLiftsToBottom();
      bot.slideSystem.releaseClamp();
      bot.slideSystem.rotateFourBarToGrab();
    }
  }
}
