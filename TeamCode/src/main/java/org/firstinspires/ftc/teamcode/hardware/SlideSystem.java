package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import com.andoverrobotics.core.utilities.CachedMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ConfigUser;

import java.util.Arrays;

public class SlideSystem extends ConfigUser<SlideSystem.Config> {
  public static class Config {
    public double slideSpeed;
    public double clampClosedPosition, clampReleasePosition, clampOpenPosition, clampSpeed;
    public double fourBarFullyInPosition, fourBarFullyOutPosition, fourBarGrabPosition, fourBarReleasePosition, fourBarSpeed;

    public int ticksPerLevel, intakeLiftHeightTicks, liftLevelOffset;

    public int liftBottomTicks, liftTopTicks;
  }

  public CachedMotor liftLeft, liftRight;
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

  public void prepareToIntake() {
    setLiftTargetPosition(config.intakeLiftHeightTicks);
    runLiftsToTargetPosition(1);
    openClamp();
    rotateFourBarToGrab();
  }

  public void setLiftTargetPosition(int target) {
    liftLeft.setTargetPosition(target);
    liftRight.setTargetPosition(target);
  }

  public void zeroLifts() {
    for (CachedMotor motor : Arrays.asList(liftLeft, liftRight)) {
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
  }

  public void setLiftPower(double power) {
    if (willLiftExceedLimitsGivenPower(power)) return;
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

  public boolean willLiftExceedLimitsGivenPower(double power) {
    int leftPos = liftLeft.getMotor().getCurrentPosition(),
        rightPos = liftRight.getMotor().getCurrentPosition(),
        minPos = Math.min(leftPos, rightPos),
        maxPos = Math.max(leftPos, rightPos);

    return (power < 0 && minPos <= config.liftBottomTicks) || (power > 0 && maxPos >= config.liftTopTicks);
  }

  public void startAligningLiftSets() {
    int leftPos = liftLeft.getMotor().getCurrentPosition(),
        rightPos = liftRight.getMotor().getCurrentPosition(),
        targetPos = Range.clip((leftPos + rightPos) / 2, config.liftBottomTicks, config.liftTopTicks);

    setLiftTargetPosition(targetPos);
    runLiftsToTargetPosition(1);
  }

  public void startRunningLiftsToLevel(int level) {
    int targetPosition = liftTarget(level);
    Log.v("SlideSystem", "lift target is " + targetPosition);
    setLiftTargetPosition(targetPosition);
    runLiftsToTargetPosition(1);
  }

  public void startRunningLiftsToBottom() {
    setLiftTargetPosition(0);
    runLiftsToTargetPosition(1);
  }

  public boolean isLiftBusy() {
    return liftLeft.getMotor().isBusy() || liftRight.getMotor().isBusy();
  }

  private int liftTarget(int level) {
    if (level == 0) {
      return config.intakeLiftHeightTicks;
    }
    return Math.max((level - 1) * config.ticksPerLevel + config.liftLevelOffset, 1000);
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

  public void rotateFourBarToTop() {
    fourBar.setPosition(0);
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
