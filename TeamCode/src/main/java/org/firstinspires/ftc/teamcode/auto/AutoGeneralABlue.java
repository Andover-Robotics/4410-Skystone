package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "4410-2020 Auto A Blue", group = "Competition")
public class AutoGeneralABlue extends AutoGeneralA {
  @Override
  public void runOpMode() {
    runForColor(AllianceColor.BLUE);
  }
}
