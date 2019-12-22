package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "4410-2020 Auto D Blue", group = "Competition")
public class AutoGeneralDBlue extends AutoGeneralD {
  @Override
  public void runOpMode() {
    initFields();
    waitForStart();

    alliance = AllianceColor.BLUE;
    runForCurrentAlliance();
  }

}
