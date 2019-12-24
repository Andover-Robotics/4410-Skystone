package org.firstinspires.ftc.teamcode.selfcheck.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.openftc.revextensions2.ExpansionHubEx;

public class RobotBatteryTest extends SystemTest {
  @Override
  public String systemName() {
    return "Robot battery";
  }

  @Override
  public void checkSystem(OpMode opMode) throws IllegalStateException {
    ExpansionHubEx hub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    double voltage = hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);

    assertThat(voltage > 13.0,
        "Robot battery voltage (%.2f) not greater than 13V", voltage);
  }
}
