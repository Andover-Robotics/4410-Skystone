package org.firstinspires.ftc.teamcode.selfcheck.tests;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class SystemTest {
  public abstract String systemName();
  public abstract void checkSystem(OpMode opMode) throws IllegalStateException;

  public void assertThat(boolean condition, String message, Object... varargs) {
    if (!condition) {
      String error = String.format(message, varargs);
      Log.e("SystemTest: " + systemName(), "assertion failed: " + error);
      throw new IllegalStateException(error);
    }
  }
}
