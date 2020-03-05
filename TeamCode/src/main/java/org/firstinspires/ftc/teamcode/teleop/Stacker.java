package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Bot;

/**
 * Facilitates stacking by memorizing the current level, etc.
 */
public class Stacker {
  private static final double TICKS_PER_REV = 383.6;
  private static final double LEVEL_HEIGHT = 4.5, SPOOL_CIRCUMFERENCE = 38 / DistanceUnit.mmPerInch * Math.PI;
  private static final int LEVEL_2_TICKS = 150;

  // ticks/rev * rev/in = ticks/in
  private static final double kTicksPerLevel = TICKS_PER_REV / SPOOL_CIRCUMFERENCE * LEVEL_HEIGHT;

  private final Bot bot;
  private int level = 0;

  public Stacker(Bot bot) {
    this.bot = bot;
  }

  public int getLevel() {
    return level;
  }

  public void goToNextLevel() {
    if (level < 8) level++;
    runToLevel();
  }

  public void goToSameLevel() {
    runToLevel();
  }

  public void goBackToLevelZero() {
    level = 0;
    runToLevel();
  }

  private void runToLevel() {
    int ticks = ticksForLevel(level);
    bot.slideSystem.setLiftTargetPosition(ticks);
    bot.slideSystem.runLiftsToTargetPosition(0.9);

    if (level == 0) {
      bot.slideSystem.rotateFourBarFullyOut();
    } else if (level == 1) {
      bot.slideSystem.rotateFourBarToRelease();
    } else {
      bot.slideSystem.rotateFourBarToIntermediate();
    }
  }

  private int ticksForLevel(int level) {
    if (level == 0) {
      return 50;
    }
    if (level == 1) {
      return 0;
    }
    return (int) Math.round((level - 2) * kTicksPerLevel + LEVEL_2_TICKS);
  }
}
