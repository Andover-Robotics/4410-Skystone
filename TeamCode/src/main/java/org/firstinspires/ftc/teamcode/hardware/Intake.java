package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.time.Clock;

public class Intake {
  private long pulseStartMillis = System.currentTimeMillis();
  private final DcMotor leftWheel, rightWheel;

  public Intake(DcMotor flyWheelLeft, DcMotor flyWheelRight) {
    leftWheel = flyWheelLeft;
    rightWheel = flyWheelRight;

    leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  private void spin(double power) {
    leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    leftWheel.setPower(power);
    rightWheel.setPower(power);
  }

  public void startPulse() {
    pulseStartMillis = System.currentTimeMillis();
  }

  public void pulse(double power, double variance, double hz) {
    long dt = System.currentTimeMillis() - pulseStartMillis;
    spin(Math.cos(dt * 2*Math.PI * hz) * variance + power);
  }

  public void takeIn(double power) {
    spin(Math.abs(power));
  }

  public void takeOut(double power) {
    spin(-Math.abs(power));
  }

  public void stop() {
    spin(0);
  }
}
