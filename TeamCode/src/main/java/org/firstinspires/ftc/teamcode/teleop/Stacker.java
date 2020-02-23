package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Bot;

/**
 * Facilitates stacking by memorizing the current level, etc.
 */
public class Stacker {
  private static final double TICKS_PER_REV = 537.6;
  private static final double LEVEL_HEIGHT = 4, SPOOL_CIRCUMFERENCE = 38 / DistanceUnit.mmPerInch * Math.PI;

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
    if (level < 9) level++;
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
    // To be extra sure, we send two identical commands for redundancy...?
    bot.slideSystem.setLiftTargetPosition((int) (kTicksPerLevel * level));
    bot.slideSystem.setLiftTargetPosition((int) (kTicksPerLevel * level));
    bot.slideSystem.runLiftsToTargetPosition(0.9);

    if (level == 0) {
      bot.slideSystem.rotateFourBarFullyOut();
    } else {
      bot.slideSystem.rotateFourBarToRelease();
    }
  }
}
