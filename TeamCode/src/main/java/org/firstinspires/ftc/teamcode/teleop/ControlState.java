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

    PICKUP((gamepad, bot) -> {

    }, s -> {
    }, s -> s.a, Color.YELLOW),

    DELIVERY((gamepad, bot) -> {

    }, s -> {
    }, s -> s.b, 0xFFFF8000 /* orange */),

    STACKING((gamepad, bot) -> {

    }, s -> {
    }, s -> s.y, Color.MAGENTA),

    RETURN((gamepad, bot) -> {

    }, s -> {
    }, s -> s.x, Color.CYAN),

    MANUAL((gamepad, bot) -> {

      // left y: lift
      if (Math.abs(gamepad.left_stick_y) > 0.1) {
        bot.slideSystem.setLiftPower(-gamepad.left_stick_y);
      } else {
        bot.slideSystem.holdLiftHeight();
      }

      // right y: clamp
//      bot.slideSystem.setClampSpeed(-gamepad.right_stick_y);
      if (gamepad.right_stick_y < -0.6) {
        bot.slideSystem.openClamp();
      }
      else if (gamepad.right_stick_y > 0.6) {
        bot.slideSystem.closeClamp();
      }

      // right x: four bar
      bot.slideSystem.setFourBarSpeed(gamepad.right_stick_x);

    }, s -> {}, g -> g.right_bumper, Color.GRAY);

    private final BiConsumer<Gamepad, Bot> update;
    private final Consumer<Bot> justEntered;
    private final Predicate<Gamepad> shouldEnter;
    private final int modeColor;

    Stage(BiConsumer<Gamepad, Bot> update, Consumer<Bot> justEntered,
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

  public static void runLoop(OpMode opMode) {
    currentStage.update.accept(opMode.gamepad2, Bot.getInstance());
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