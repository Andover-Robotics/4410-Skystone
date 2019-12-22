package org.firstinspires.ftc.teamcode.hardware;

import com.andoverrobotics.core.utilities.CachedMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ConfigUser;

import java.util.Arrays;

public class SlideSystem extends ConfigUser<SlideSystem.Config> {
  public static class Config {
    public double slideSpeed;
    public int slidePositionIncrement;
    public double clampClosedPosition, clampOpenPosition, clampSpeed;
    public double fourBarSpeed;
  }

  private CachedMotor liftLeft, liftRight;
  private Servo clamp;
  private CRServo fourBar;

  public SlideSystem(DcMotor liftLeft, DcMotor liftRight, Servo clamp, CRServo fourBar) {
    super("slideSystem.properties", new Config());
    this.liftLeft = new CachedMotor(liftLeft);
    this.liftRight = new CachedMotor(liftRight);
    this.clamp = clamp;
    this.fourBar = fourBar;

    // More predictable
    setupLiftMotors(liftLeft, liftRight);

    clamp.scaleRange(config.clampOpenPosition, config.clampClosedPosition);
  }

  private void setupLiftMotors(DcMotor liftLeft, DcMotor liftRight) {
    liftLeft.setTargetPosition(0);
    liftRight.setTargetPosition(0);

    liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void zeroLifts() {
    for (CachedMotor motor : Arrays.asList(liftLeft, liftRight)) {
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
  }

  public void setLiftPower(double power) {
    liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    liftLeft.setPower(power * config.slideSpeed);
    liftRight.setPower(power * config.slideSpeed);
    liftHeld = false;
  }

  private boolean liftHeld = false;
  public void holdLiftHeight() {
    if (!liftHeld) {
      for (CachedMotor motor : Arrays.asList(liftLeft, liftRight)) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(liftLeft.getMotor().getCurrentPosition());
        motor.setPower(0.4);
      }
      liftHeld = true;
    }
  }

  public void openClamp() {
    setClampPosition(0);
  }

  public void closeClamp() {
    setClampPosition(1);
  }

  public void setClampPosition(double position) {
    clamp.setPosition(position);
  }

  public void setClampSpeed(double speed) {
    clamp.setPosition(clamp.getPosition() + speed * config.clampSpeed);
  }

  public void setFourBarSpeed(double speed) {
    fourBar.setPower(speed * config.fourBarSpeed);
  }
}
