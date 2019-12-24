package org.firstinspires.ftc.teamcode.selfcheck.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

public class MotorEncoderTest extends SystemTest {
  private static final int WIGGLE_TIMEOUT_MS = 400;

  @Override
  public String systemName() {
    return "All motor encoders";
  }

  @Override
  public void checkSystem(OpMode opMode) throws IllegalStateException {
    for (String name : Arrays.asList("motorFL", "motorFR", "motorBL", "motorBR", "liftL", "liftR")) {
      checkMotor(opMode.hardwareMap.dcMotor.get(name), name);
    }
  }

  private void checkMotor(DcMotor motor, String name) {
    long startTime = System.currentTimeMillis();

    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    while (motor.getCurrentPosition() != 0);

    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setPower(0.5);

    while (motor.getCurrentPosition() < 10) {
      if (System.currentTimeMillis() - startTime >= WIGGLE_TIMEOUT_MS) {
        motor.setPower(0);
        throw new IllegalStateException("motor with name " + name + " failed encoder test. Diagnose manually");
      }
    }
    motor.setPower(0);
  }
}
