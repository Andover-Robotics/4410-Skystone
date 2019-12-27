package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Bot;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class ControlState {

  public enum Stage {

    PICKUP((bot, opMode) -> {
      TeleOpMain.driveSpeed = 1;
      if (opMode.gamepad2.right_bumper) {
        // clamp
        bot.slideSystem.closeClamp();
      }
    }, s -> {
    }, s -> s.a && !s.start, Color.YELLOW),

    DELIVERY((bot, opMode) -> {
      TeleOpMain.driveSpeed = 1;

//      if (opMode.gamepad2.right_trigger > 0.3) {
//        bot.slideSystem.startRunningLiftsToLevel(ControlState.stoneLevel);
//      }

    }, s -> {
    }, s -> s.x && !s.start, 0xFFFF8000 /* orange */),

    STACKING((bot, opMode) -> {
      TeleOpMain.driveSpeed = 0.4;

      bot.slideSystem.setLiftPower(opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger);
      if (opMode.gamepad2.left_bumper) {
        bot.slideSystem.releaseClamp();
      }

    }, bot -> {
      bot.slideSystem.rotateFourBarToRelease();
    }, s -> s.y && !s.start, Color.MAGENTA),

    RETURN((bot, opMode) -> {
      TeleOpMain.driveSpeed = 1;

    }, bot -> {
      // Reset 4-bar and clamp
      bot.slideSystem.rotateFourBarToGrab();
      bot.slideSystem.openClamp();
      //bot.slideSystem.startRunningLiftsToLevel(0);
    }, s -> s.b && !s.start, Color.CYAN),

    MANUAL((bot, opMode) -> {

      // left y: lift
      if (Math.abs(opMode.gamepad2.left_stick_y) > 0.1) {
        bot.slideSystem.setLiftPower(-opMode.gamepad2.left_stick_y);
      } else {
        bot.slideSystem.holdLiftHeight();
      }

      // right y: clamp
//      bot.slideSystem.setClampSpeed(-gamepad.right_stick_y);
      if (opMode.gamepad2.right_stick_y < -0.2) {
        bot.slideSystem.setClampSpeed(opMode.gamepad2.right_stick_y);
      }
      else if (opMode.gamepad2.right_stick_y > 0.4) {
        bot.slideSystem.closeClamp();
      }

      // right x: four bar
      bot.slideSystem.setFourBarSpeed(opMode.gamepad2.right_stick_x);

    }, bot -> {
      bot.slideSystem.relaxLift();
    }, g -> g.right_bumper, Color.GRAY),

    ONE_CTRL_MANUAL((bot, opMode) -> {
      Gamepad pad = opMode.gamepad1;

      // dpad up/down: lift
      if (pad.dpad_up) {
        bot.slideSystem.setLiftPower(0.7);
      } else if (pad.dpad_down) {
        bot.slideSystem.setLiftPower(-0.7);
      } else {
        bot.slideSystem.holdLiftHeight();
      }

      // X & B: four-bar
      if (pad.x) {
        bot.slideSystem.rotateFourBarToGrab();
      } else if (pad.b) {
        bot.slideSystem.rotateFourBarToRelease();
      }

      // Y & A & back: clamp
      if (pad.a) {
        bot.slideSystem.closeClamp();
      } else if (pad.y) {
        bot.slideSystem.openClamp();
      } else if (pad.back) {
        bot.slideSystem.releaseClamp();
      }

    }, bot -> {
      bot.slideSystem.relaxLift();
    }, g -> g.left_stick_button && g.right_stick_button, Color.RED);

    private final BiConsumer<Bot, TeleOpMain> update;
    private final Consumer<Bot> justEntered;
    private final Predicate<Gamepad> shouldEnter;
    private final int modeColor;

    Stage(BiConsumer<Bot, TeleOpMain> update, Consumer<Bot> justEntered,
          Predicate<Gamepad> shouldEnter, int modeColor) {
      this.update = update;
      this.justEntered = justEntered;
      this.shouldEnter = shouldEnter;
      this.modeColor = modeColor;
    }
  }

  public static short stoneLevel = 1;
  public static Stage currentStage = Stage.MANUAL;

  public static void updateStage(OpMode opMode) {
    for (Stage stage : Stage.values()) {
      if (stage.shouldEnter.test(opMode.gamepad2)) {
        switchToStage(stage);
      }
    }
  }

  public static void switchToStage(Stage stage) {
    currentStage = stage;
    showColor(stage.modeColor);
    stage.justEntered.accept(Bot.getInstance());
  }

  public static void runLoop(TeleOpMain opMode) {
    currentStage.update.accept(Bot.getInstance(), opMode);
  }

  private static void showColor(int color) {
    Bot bot = Bot.getInstance();
    int r = ((color >> 16) & 0xff);
    int g = ((color >> 8) & 0xff);
    int b = ((color) & 0xff);
    bot.hub1.setLedColor(r, g, b);
    bot.hub2.setLedColor(r, g, b);
  }
}