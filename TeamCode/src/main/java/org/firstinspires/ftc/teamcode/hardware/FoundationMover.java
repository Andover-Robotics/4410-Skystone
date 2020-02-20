package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ConfigUser;

public class FoundationMover extends ConfigUser<FoundationMover.ConfigSchema> {

  public static class ConfigSchema {
    public boolean leftReverse;
    public int leftOperatingRange, rightOperatingRange;
  }

  public Servo armLeft, armRight;

  public FoundationMover(Servo armLeft, Servo armRight) {
    super("foundationMover.properties", new ConfigSchema());

    this.armLeft = armLeft;
    this.armRight = armRight;

    if (config.leftReverse) {
      armLeft.setDirection(Servo.Direction.REVERSE);
    } else {
      armRight.setDirection(Servo.Direction.REVERSE);
    }
  }

  public void armDown() {
    armLeft.setPosition(convertDegrees(0, config.leftOperatingRange));
    armRight.setPosition(convertDegrees(0, config.rightOperatingRange));
  }


  public void armUp() {
    armLeft.setPosition(convertDegrees(120, config.leftOperatingRange));
    armRight.setPosition(convertDegrees(120, config.rightOperatingRange));
  }

  private double convertDegrees(int degrees, int operatingRange) {
    return (double) degrees / operatingRange;
  }
}
