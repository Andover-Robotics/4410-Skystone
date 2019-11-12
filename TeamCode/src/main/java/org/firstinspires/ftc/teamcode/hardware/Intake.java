package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

  private final DcMotor leftWheel, rightWheel;

  public Intake(DcMotor flyWheelLeft, DcMotor flyWheelRight) {
    leftWheel = flyWheelLeft;
    rightWheel = flyWheelRight;

    rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  private void spin(double power) {
    leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    leftWheel.setPower(power);
    rightWheel.setPower(power);
  }

  public void takeIn(double power) {
    spin(Math.abs(power));
  }

  public void takeOut(double power) {
    spin(-Math.abs(power));
  }
}
