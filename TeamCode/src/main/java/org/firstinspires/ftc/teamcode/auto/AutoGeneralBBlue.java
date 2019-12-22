package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "4410-2020 Auto B Blue", group = "Competition")
public class AutoGeneralBBlue extends AutoGeneralB {
  @Override
  public void runOpMode() throws InterruptedException {
    initFields();
    waitForStart();

    alliance = AllianceColor.BLUE;
    runForCurrentAlliance();
  }
}
