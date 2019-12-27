package org.firstinspires.ftc.teamcode.hardware;

import com.andoverrobotics.core.utilities.CachedMotor;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.ConfigUser;

import java.util.Arrays;

public class SlideSystem extends ConfigUser<SlideSystem.Config> {
  public static class Config {
    public double slideSpeed;
    public int slidePositionIncrement;
    public double clampClosedPosition, clampReleasePosition, clampOpenPosition, clampSpeed;
    public double fourBarFullyInPosition, fourBarFullyOutPosition, fourBarGrabPosition, fourBarReleasePosition, fourBarSpeed;

    public int ticksPerLevel, intakeLiftHeightTicks, liftLevelOffset;
  }

  private CachedMotor liftLeft, liftRight;
  private Servo clamp;
  private Servo fourBar;

  public SlideSystem(DcMotor liftLeft, DcMotor liftRight, Servo clamp, Servo fourBar) {
    super("slideSystem.properties", new Config());
    this.liftLeft = new CachedMotor(liftLeft);
    this.liftRight = new CachedMotor(liftRight);
    this.clamp = clamp;
    this.fourBar = fourBar;

    liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

    // More predictable
    setupLiftMotors(liftLeft, liftRight);

    clamp.scaleRange(config.clampOpenPosition, config.clampClosedPosition);
    fourBar.scaleRange(config.fourBarFullyInPosition, config.fourBarFullyOutPosition);
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

  public void runLiftsToTargetPosition(double power) {
    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    liftLeft.setPower(power * config.slideSpeed);
    liftRight.setPower(power * config.slideSpeed);
    liftHeld = false;
  }

  public void startRunningLiftsToLevel(int level) {
    int targetPosition = liftTarget(level);
    liftLeft.setTargetPosition(targetPosition);
    liftRight.setTargetPosition(targetPosition);
    runLiftsToTargetPosition(config.slideSpeed);
  }

  private int liftTarget(int level) {
    if (level == 0) {
      return config.intakeLiftHeightTicks;
    }
    return (level - 1) * config.ticksPerLevel + config.liftLevelOffset;
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

  public void relaxLift() {
    for (CachedMotor motor : Arrays.asList(liftLeft, liftRight)) {
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      motor.setPower(0);
    }
  }

  public void openClamp() {
    setClampPosition(0);
  }

  public void releaseClamp() {
    setClampPosition(config.clampReleasePosition);
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
    fourBar.setPosition(fourBar.getPosition() + speed * config.fourBarSpeed);
  }

  public void rotateFourBarToGrab() {
    fourBar.setPosition(config.fourBarGrabPosition);
  }

  public void rotateFourBarToRelease() {
    fourBar.setPosition(config.fourBarReleasePosition);
  }

  public PIDFCoefficients getLiftCoefficients() {
    return ((DcMotorEx) liftLeft.getMotor()).getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void setLiftCoefficients(PIDFCoefficients newCoefficients) {
    for (CachedMotor motor : Arrays.asList(liftLeft, liftRight)) {
      ((DcMotorEx) motor.getMotor()).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newCoefficients);
    }
  }
}
