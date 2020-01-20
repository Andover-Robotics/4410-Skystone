package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ConfigUser;

public class SideClaw extends ConfigUser<SideClaw.Configuration> {
  public static class Configuration {
    public double armUpPosition, armDownPosition;
  }

  private Servo arm;

  public SideClaw(Servo arm) {
    super("sideClaw.properties", new Configuration());
    this.arm = arm;
  }

  public void armUp() {
    arm.setPosition(config.armUpPosition);
  }

  public void armDown() {
    arm.setPosition(config.armDownPosition);
  }

  public void clamp() {
    // do nothing
  }

  public void release() {
    // do nothing
  }
}
