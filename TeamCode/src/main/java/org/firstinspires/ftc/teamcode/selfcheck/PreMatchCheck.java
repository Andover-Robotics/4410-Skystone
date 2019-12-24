package org.firstinspires.ftc.teamcode.selfcheck;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.selfcheck.tests.MotorEncoderTest;
import org.firstinspires.ftc.teamcode.selfcheck.tests.PhoneBatteryTest;
import org.firstinspires.ftc.teamcode.selfcheck.tests.RobotBatteryTest;
import org.firstinspires.ftc.teamcode.selfcheck.tests.SystemTest;

import java.util.Arrays;
import java.util.stream.Stream;

@Autonomous(name = "Pre-Match Check", group = "Competition")
public class PreMatchCheck extends LinearOpMode {
  private SystemTest[] tests = {
      new MotorEncoderTest(),
      new PhoneBatteryTest(),
      new RobotBatteryTest()
  };

  @Override
  public void runOpMode() {

    telemetry.setAutoClear(false);
    telemetry.addData("Tests", Arrays.toString(Stream.of(tests).map(SystemTest::systemName).toArray()));
    telemetry.update();

    waitForStart();

    for (SystemTest test : tests) {
      telemetry.addData("Testing", test.systemName());
      telemetry.update();

      try {

        test.checkSystem(this);
        telemetry.addData("Test", "Passed ✅");

      } catch (IllegalStateException failure) {
        telemetry.addData("Test Failed ❌", failure.getMessage());
      }

      telemetry.update();
    }

    while (!isStopRequested()) {
      idle();
    }
  }
}
