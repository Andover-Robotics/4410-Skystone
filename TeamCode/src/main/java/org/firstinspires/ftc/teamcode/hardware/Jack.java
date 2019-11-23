package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Jack {
  private DcMotor left, right;

  public Jack(DcMotor left, DcMotor right) {
    this.left = left;
    this.right = right;

    left.setDirection(DcMotorSimple.Direction.REVERSE);
    left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void setPower(double power) {
    left.setPower(power);
    right.setPower(power);
  }
}
