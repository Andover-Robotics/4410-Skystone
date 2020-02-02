package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ConfigUser;

public class SideClaw extends ConfigUser<SideClaw.Configuration> {
  public static class Configuration {
    public double armUpPosition, armDownPosition, armDisabledPosition;
    public double clampClosedPosition, clampOpenPosition, clampFoldedPosition;
  }

  private Servo arm, clamp;

  public SideClaw(Servo arm, Servo clamp) {
    super("sideClaw.properties", new Configuration());
    this.arm = arm;
    this.clamp = clamp;
  }

  public void armDisabled() {
    arm.setPosition(config.armDisabledPosition);
  }

  public void armUp() {
    arm.setPosition(config.armUpPosition);
  }

  public void armDown() {
    arm.setPosition(config.armDownPosition);
  }

  public void clamp() {
    clamp.setPosition(config.clampClosedPosition);
  }

  public void release() {
    clamp.setPosition(config.clampOpenPosition);
  }

  public void foldClamp() {
    clamp.setPosition(config.clampFoldedPosition);
  }
}
