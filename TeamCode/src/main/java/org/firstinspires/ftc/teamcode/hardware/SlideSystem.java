package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public boolean debugControlDashboard;
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

    clamp.scaleRange(config.clampClosedPosition, config.clampOpenPosition);
    fourBar.scaleRange(config.fourBarFullyInPosition, config.fourBarFullyOutPosition);
  }

  private void setupLiftMotors(DcMotor liftLeft, DcMotor liftRight) {
    for (DcMotor motor : Arrays.asList(liftLeft, liftRight)) {
      DcMotorEx motorEx = (DcMotorEx) motor;

      motorEx.setTargetPosition(0);
      motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorEx.setTargetPositionTolerance(15);

      PIDFCoefficients coefficients = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
      motorEx.setPositionPIDFCoefficients(coefficients.p + 1.1);

      coefficients = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
      motorEx.setVelocityPIDFCoefficients(coefficients.p + 0.1, coefficients.i + 0.5, coefficients.d, coefficients.f + 0.2);
    }
  }

  public void prepareToIntake() {
    relaxLift();
//    setLiftTargetPosition(config.intakeLiftHeightTicks);
//    runLiftsToTargetPosition(1);
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
      motor.setTargetPosition(0);
      motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
  }

  public void setLiftPower(double power) {
    if (willLiftExceedLimitsGivenPower(power)) {
      relaxLift();
      return;
    }
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

  public void startRunningLiftsToBottom() {
    relaxLift();
    setLiftTargetPosition(0);
    runLiftsToTargetPosition(1);
  }

  public boolean isLiftRunningToPosition() {
//    return liftLeft.getMotor().isBusy() || liftRight.getMotor().isBusy();
    int leftPos = liftLeft.getMotor().getCurrentPosition(),
        rightPos = liftRight.getMotor().getCurrentPosition(),
        leftError = Math.abs(leftPos - liftLeft.getTargetPosition()),
        rightError = Math.abs(rightPos - liftRight.getTargetPosition());

    return Math.max(leftError, rightError) > 50;
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
    liftHeld = false;
  }

  public void relayLiftDebugDashboard() {
    if (config.debugControlDashboard) {
      TelemetryPacket packet = new TelemetryPacket();
      packet.put("leftLift current", liftLeft.getMotor().getCurrentPosition());
      packet.put("rightLift current", liftRight.getMotor().getCurrentPosition());
      packet.put("leftLift target", liftLeft.getTargetPosition());
      packet.put("rightLift target", liftRight.getTargetPosition());

      FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
  }

  public void openClamp() {
    setClampPosition(1);
  }

  public void releaseClamp() {
    setClampPosition(config.clampReleasePosition);
  }

  public void closeClamp() {
    setClampPosition(0);
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

  public void rotateFourBarFullyIn() {
    fourBar.setPosition(0);
  }

  public void rotateFourBarFullyOut() {
    fourBar.setPosition(1);
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
