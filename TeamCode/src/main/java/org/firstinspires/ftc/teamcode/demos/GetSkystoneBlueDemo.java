package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.CVSkystoneDetector;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "Get Skystone Blue", group = "Autonomous")
public class GetSkystoneBlueDemo extends GetSkystoneDemo {
  public void runOpMode() {
    super.runOpMode(AllianceColor.BLUE);
  }
}
