package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Facilitates stacking by memorizing the current level, etc.
 */
public class Stacker {
  private final Bot bot;
  private int level = 0;

  public Stacker(Bot bot) {
    this.bot = bot;
  }

  public int getLevel() {
    return level;
  }

  public void goToNextLevel() {
    if (level < 6) level++;
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
    bot.slideSystem.setLiftTargetPosition(150 * level);
    bot.slideSystem.setLiftTargetPosition(150 * level);
    bot.slideSystem.runLiftsToTargetPosition(0.5);

    if (level == 0) {
      bot.slideSystem.rotateFourBarFullyOut();
    } else {
      bot.slideSystem.rotateFourBarToRelease();
    }
  }
}
