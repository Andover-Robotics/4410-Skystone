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

  public CRServo armLeft, armRight;

  public FoundationMover(CRServo armLeft, CRServo armRight) {
    super("foundationMover.properties", new ConfigSchema());

    this.armLeft = armLeft;
    this.armRight = armRight;

//    armRight.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void setPower(double power) {
    armLeft.setPower(power);
    armRight.setPower(-power);
  }

  public void armDown() {
    setPower(1);
    try {
      Thread.sleep(800);
    } catch (InterruptedException e) {
      return;
    }
    setPower(0);
  }


  public void armUp() {
    setPower(-1);
    try {
      Thread.sleep(800);
    } catch (InterruptedException e) {
      return;
    }
    setPower(0);
  }
}
