package org.firstinspires.ftc.teamcode.teleop;

import com.andoverrobotics.core.utilities.Coordinate;
import com.andoverrobotics.core.utilities.InputColumnResponder;
import com.andoverrobotics.core.utilities.InputColumnResponderImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.List;

@TeleOp(name = "4410-2020 TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
  private Bot bot;
  public static double driveSpeed = 1;
  public static int fieldCentricDelta = 0;
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
    stacker = new Stacker(bot);
    stackerIcr = new InputColumnResponderImpl();
    stackerIcr
        .register(() -> gamepad2.dpad_up, () -> {
          liftHoldDesired = false;
          stacker.goToNextLevel();
        })
        .register(() -> gamepad2.dpad_down, () -> {
          liftHoldDesired = false;
          stacker.goBackToLevelZero();
        })
        .register(() -> gamepad2.dpad_right, () -> {
          liftHoldDesired = false;
          stacker.goToSameLevel();
        });

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

  public void start() {
    bot.slideSystem.rotateFourBarToGrab();
    bot.slideSystem.openClamp();
    bot.sideClaw.armDisabled();
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
    controlIntakePulse();

    automateSimply();
    controlStacker();
    bot.slideSystem.relayLiftDebugDashboard();

    showLiftStatus();

    telemetry.addData("cycle period", "%d ms", System.currentTimeMillis() - startMillis);
    telemetry.addData("stacker level", "%d", stacker.getLevel());
    telemetry.addData("drive speed", "%.3f", driveSpeed);
    telemetry.addData("current draw 1", bot.hub1.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
    telemetry.addData("current draw 2", bot.hub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));


    if (gamepad1.right_stick_button) {
      // Reset field centric (set current heading to human heading)
      fieldCentricDelta = (int) bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
  }

  private void controlSideClaw() {
    if (gamepad1.y) {
      bot.sideClaw.armRelease();
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
    double outtakeSpeed = gamepad1.right_trigger * 0.5;

    if (intakeSpeed > 0) {
      bot.intake.takeIn(intakeSpeed);
    } else if (outtakeSpeed > 0) {
      bot.intake.takeOut(outtakeSpeed);
    } else if (!intakePulseDesired) {
      bot.intake.stop();
    }
  }

  private void controlFoundationMovers() {
    if (gamepad1.left_bumper) {
      bot.foundationMover.armDown();
    } else if (gamepad1.right_bumper) {
      bot.foundationMover.armUp();
    }
  }

  private void adjustDriveSpeed() {
    if (gamepad1.back) {
      driveSpeed = 0.35;
    } else if (gamepad1.start) {
      driveSpeed = 1;
    }
  }

  private boolean intakePulseDesired = false;

  private void controlIntakePulse() {
    if (gamepad1.a) {
      intakePulseDesired = true;
    } else if (bot.loadSensor.stonePresent() || gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
      intakePulseDesired = false;
    }

    if (intakePulseDesired) {
      bot.intake.pulse(0.4, 0.3, 1.5);
    }
  }

  private Coordinate getDpadVector(Gamepad g) {
    int x = 0, y = 0;
    if (g.dpad_up) y++;
    if (g.dpad_down) y--;
    if (g.dpad_right) x++;
    if (g.dpad_left) x--;

    return Coordinate.fromXY(x, y);
  }

  private void driveMovement() {
    Coordinate driveVector = Coordinate.fromXY(gamepad1.left_stick_x, -gamepad1.left_stick_y);

    if (useFieldCentric)
      driveVector = driveVector.rotate(
          (int) -bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle + fieldCentricDelta);

    driveVector = driveVector.add(getDpadVector(gamepad1).scale(0.4));

    double microMultiplier = driveSpeed;
    double rotation = gamepad1.right_stick_x * driveSpeed;

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      bot.driveTrain.setRotationPower(rotation);
    } else {
      bot.driveTrain.setStrafeRotation(driveVector,
          Range.clip(driveVector.getPolarDistance() * microMultiplier, 0, 1), rotation);
    }
  }

  private boolean liftHoldDesired = false;

  private void automateSimply() {

    // left y: lift
    if (Math.abs(gamepad2.left_stick_y) > 0.1) {
      liftHoldDesired = true;
      bot.slideSystem.setLiftPower(-gamepad2.left_stick_y);
    } else if (liftHoldDesired) {
      // Not necessary for current stringing scheme; might break the key
//      bot.slideSystem.holdLiftHeight();
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
      bot.slideSystem.setClampSpeed(-gamepad2.right_stick_y);
    } else if (gamepad2.right_stick_y > 0.3) {
      bot.slideSystem.closeClamp();
    } else if (gamepad2.back) {
      bot.slideSystem.releaseClamp();
    }

    // right x: four bar
    bot.slideSystem.setFourBarSpeed(gamepad2.right_stick_x);

    if (!gamepad2.start) {
      if (gamepad2.a) {
        // Pickup
        bot.slideSystem.rotateFourBarFullyIn();
        bot.slideSystem.closeClamp();
      }
      if (gamepad2.x) {
        // Stacking level 0
        bot.slideSystem.rotateFourBarFullyOut();
      }
      if (gamepad2.y) {
        // Stacking level 2+
        bot.slideSystem.rotateFourBarToRelease();
      }
      if (gamepad2.b) {
        // Return & prep for pickup
        bot.slideSystem.openClamp();
        bot.slideSystem.startRunningLiftsToBottom();
        bot.slideSystem.rotateFourBarToGrab();
        liftHoldDesired = false;
      }
    }

    addDiagnosticData("automation", "liftHoldDesired? %s", liftHoldDesired ? "yes" : "no");
  }

  private Stacker stacker;
  private InputColumnResponder stackerIcr;

  private void controlStacker() {
    stackerIcr.update();
  }
}
