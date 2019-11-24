package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "4410-2020 Auto Blue", group = "Competition")
public class AutoBlue extends AutoMain {
  @Override
  public void runOpMode() {
    runForColor(AllianceColor.BLUE);
  }
}
