package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class FoundationMover {
  private Servo armLeft, armRight;

  public FoundationMover(Servo armLeft, Servo armRight) {
    this.armLeft = armLeft;
    this.armRight = armRight;

    armLeft.setDirection(Servo.Direction.REVERSE);
    armLeft.scaleRange(0.66, 1);
    armRight.scaleRange(0.66, 1);
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
