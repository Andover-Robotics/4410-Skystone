package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ConfigUser;

public class FoundationMover extends ConfigUser<FoundationMover.ConfigSchema> {

  public static class ConfigSchema {
    boolean leftReverse;
    double scaleRangeMin, scaleRangeMax;
  }

  private Servo armLeft, armRight;

  public FoundationMover(Servo armLeft, Servo armRight) {
    super("foundationMover.properties", new ConfigSchema());

    this.armLeft = armLeft;
    this.armRight = armRight;

    if (config.leftReverse)
      armLeft.setDirection(Servo.Direction.REVERSE);
    else
      armRight.setDirection(Servo.Direction.REVERSE);

    armLeft.scaleRange(config.scaleRangeMin, config.scaleRangeMax);
    armRight.scaleRange(config.scaleRangeMin, config.scaleRangeMax);
  }

  public void armDown() {
    setPositions(1);
  }

  public void armUp() {
    setPositions(0);
  }

  private void setPositions(int pos) {
    armLeft.setPosition(pos);
    armRight.setPosition(pos);
  }
}
