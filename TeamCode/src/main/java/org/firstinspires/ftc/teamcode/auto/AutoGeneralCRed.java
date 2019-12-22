package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "4410-2020 Auto C Red", group = "Competition")
public class AutoGeneralCRed extends AutoGeneralC {
  @Override
  public void runOpMode() {
    initFields();
    waitForStart();

    alliance = AllianceColor.RED;
    runForCurrentAlliance();
  }

}
