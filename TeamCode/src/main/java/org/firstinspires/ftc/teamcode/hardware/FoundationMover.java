package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ConfigUser;

public class FoundationMover extends ConfigUser<FoundationMover.ConfigSchema> {

  public static class ConfigSchema {
    public boolean leftReverse;
    public double scaleRangeMin, scaleRangeMax;
  }

  public Servo armLeft, armRight;

  public FoundationMover(Servo armLeft, Servo armRight) {
    super("foundationMover.properties", new ConfigSchema());

    this.armLeft = armLeft;
    this.armRight = armRight;

    armRight.setDirection(Servo.Direction.REVERSE);
    armLeft.scaleRange(config.scaleRangeMin, config.scaleRangeMax);
    armRight.scaleRange(config.scaleRangeMin, config.scaleRangeMax);
  }

  public void armDown() {
    armLeft.setPosition(1);
    armRight.setPosition(1);
  }


  public void armUp() {
    armLeft.setPosition(0);
    armRight.setPosition(0);
  }
}
