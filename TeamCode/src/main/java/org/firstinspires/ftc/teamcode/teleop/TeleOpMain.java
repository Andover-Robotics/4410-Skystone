package org.firstinspires.ftc.teamcode.teleop;

import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.List;

@TeleOp(name = "4410-2020 TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
  private Bot bot;
  public static double driveSpeed = 1;
  private boolean useSimpleAutomation = true;
  private boolean useFieldCentric;
  private boolean diagnostics;

  private List<LynxModule> hubs;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    useFieldCentric = bot.mainConfig.getBoolean("useFieldCentric");
    diagnostics = bot.mainConfig.getBoolean("diagnostics");
    bot.slideSystem.relaxLift();
    hubs = hardwareMap.getAll(LynxModule.class);

    if (bot.mainConfig.getBoolean("useBuiltInBulkRead")) {
      setBulkReadMode(LynxModule.BulkCachingMode.MANUAL);
    } else {
      setBulkReadMode(LynxModule.BulkCachingMode.OFF);
    }

    telemetry.addData("TeleOp", "initialized");
    telemetry.addData("left lift position", bot.slideSystem.liftLeft.getMotor().getCurrentPosition());
    telemetry.addData("right lift position", bot.slideSystem.liftRight.getMotor().getCurrentPosition());
    telemetry.update();
  }

  private void setBulkReadMode(LynxModule.BulkCachingMode mode) {
    for (LynxModule module : hubs) {
      module.setBulkCachingMode(mode);
    }
  }

  public void init_loop() {
    if (gamepad1.x) {
      useSimpleAutomation = false;
      telemetry.addData("automation is now", "very complicated");
      telemetry.update();
    }
  }

  @Override
  public void loop() {
    for (LynxModule module : hubs) {
      module.clearBulkCache();
    }

    long startMillis = System.currentTimeMillis();

    adjustDriveSpeed();
    driveMovement();
    addDiagnosticData("loop timing", "passed driveMovement %d ms since start of iteration",
        System.currentTimeMillis() - startMillis);
    controlFoundationMovers();
    addDiagnosticData("loop timing", "passed controlFoundationMovers %d ms since start of iteration",
        System.currentTimeMillis() - startMillis);
    controlIntake();
    addDiagnosticData("loop timing", "passed controlIntake %d ms since start of iteration",
        System.currentTimeMillis() - startMillis);
    controlSideClaw();

    if (useSimpleAutomation) {
      automateSimply();
      bot.slideSystem.relayLiftDebugDashboard();
    } else {
      ControlState.runLoop(this);
      ControlState.updateStage(this);
    }

    showLiftStatus();

    telemetry.addData("cycle period", "%d ms", System.currentTimeMillis() - startMillis);
    telemetry.addData("current control state", ControlState.currentStage);
    telemetry.addData("drive speed", "%.3f", driveSpeed);
    telemetry.addData("current draw 1", bot.hub1.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
    telemetry.addData("current draw 2", bot.hub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));


    if (gamepad1.right_stick_button) {
      // Reset field centric (set current heading to human heading)
      bot.initImu(this);
    }
  }

  private void controlSideClaw() {
    if (gamepad1.y) {
      bot.sideClaw.armDisabled();
    }
    if (gamepad1.a) {
      bot.sideClaw.armDown();
    }
    if (gamepad1.x) {
      bot.sideClaw.clamp();
    }
    if (gamepad1.b) {
      bot.sideClaw.release();
    }
  }

  private void addDiagnosticData(String title, String content, Object... args) {
    if (diagnostics) {
      telemetry.addData(title, content, args);
    }
  }

  private boolean lastStatusBusy = false;
  private void showLiftStatus() {
    boolean liftBusy = bot.slideSystem.isLiftRunningToPosition();
    if (liftBusy != lastStatusBusy) {
      if (liftBusy) {
        bot.hub1.setLedColor(255, 50, 0);
        bot.hub2.setLedColor(255, 50, 0);
      } else {
        bot.hub1.setLedColor(0, 255, 100);
        bot.hub2.setLedColor(0, 255, 100);
      }
      lastStatusBusy = liftBusy;
    }
  }

  private void controlIntake() {
    double intakeSpeed = gamepad1.left_trigger * 0.7;
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
    if (gamepad1.left_bumper) {
      bot.foundationMover.setPower(0.6);
    } else if (gamepad1.right_bumper) {
      bot.foundationMover.setPower(-0.6);
    } else {
      bot.foundationMover.setPower(0);
    }
  }

  private void adjustDriveSpeed() {
    if (gamepad1.back) {
      driveSpeed = 0.3;
    } else if (gamepad1.start) {
      driveSpeed = 1;
    }
  }

  private void driveMovement() {
    Coordinate driveVector = Coordinate.fromXY(gamepad1.left_stick_x, -gamepad1.left_stick_y);

    if (useFieldCentric)
        driveVector = driveVector.rotate(
            (int) -bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

    double microMultiplier = driveSpeed;
    double rotation = gamepad1.right_stick_x * driveSpeed;

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      bot.driveTrain.setRotationPower(rotation);
    } else {
      bot.driveTrain.setStrafeRotation(driveVector,
          driveVector.getPolarDistance() * microMultiplier, rotation);
    }
  }

  private boolean liftHoldDesired = false;
  private void automateSimply() {

    // left y: lift
    if (Math.abs(gamepad2.left_stick_y) > 0.1) {
      liftHoldDesired = true;
      bot.slideSystem.setLiftPower(-gamepad2.left_stick_y);
    } else if (liftHoldDesired) {
      bot.slideSystem.holdLiftHeight();
    }
    if (gamepad2.right_bumper) {
      bot.slideSystem.relaxLift();
      liftHoldDesired = false;
    } else if (gamepad2.left_bumper) {
      bot.slideSystem.startAligningLiftSets();
      liftHoldDesired = false;
    }

    // right y: clamp
    if (gamepad2.right_stick_y < -0.2) {
      bot.slideSystem.setClampSpeed(gamepad2.right_stick_y);
    }
    else if (gamepad2.right_stick_y > 0.3) {
      bot.slideSystem.closeClamp();
    }
    else if (gamepad2.back) {
      bot.slideSystem.releaseClamp();
    }

    // right x: four bar
    bot.slideSystem.setFourBarSpeed(gamepad2.right_stick_x);

    if (!gamepad2.start) {
      if (gamepad2.a) {
        // Pickup
        bot.slideSystem.prepareToIntake();
        liftHoldDesired = false;
      }
      if (gamepad2.x) {
        // Delivery
        bot.slideSystem.startRunningLiftsToBottom();
        liftHoldDesired = false;
      }
      if (gamepad2.y) {
        // Stacking
        bot.slideSystem.rotateFourBarToRelease();
      }
      if (gamepad2.b) {
        // Return
        bot.slideSystem.closeClamp();
        bot.slideSystem.startRunningLiftsToBottom();
        bot.slideSystem.rotateFourBarToGrab();
        liftHoldDesired = false;
      }
    }

    addDiagnosticData("automation", "liftHoldDesired? %s", liftHoldDesired ? "yes" : "no");
  }
}
