package org.firstinspires.ftc.teamcode.selfcheck.tests;

import android.content.Context;
import android.os.BatteryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PhoneBatteryTest extends SystemTest {
  @Override
  public String systemName() {
    return "Phone batteries";
  }

  @Override
  public void checkSystem(OpMode opMode) throws IllegalStateException {
    int myBatteryLevel = getRobotControllerBatteryLevel(opMode.hardwareMap.appContext);

    assertThat(myBatteryLevel > 40,
        "robot controller battery level (%d) not above 40", myBatteryLevel);
  }

  private int getRobotControllerBatteryLevel(Context context) {
    BatteryManager manager = (BatteryManager) context.getSystemService(Context.BATTERY_SERVICE);
    return manager.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY);
  }
}
