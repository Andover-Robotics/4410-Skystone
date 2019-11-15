package org.firstinspires.ftc.teamcode.demos;

import android.util.Pair;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.auto.CVSkystoneDetector;
import org.firstinspires.ftc.teamcode.auto.StonePosition;

@Autonomous(name = "CV Skystone Detect Demo", group = "Experimental")
public class CVSkystoneDetectionDemo extends OpMode {
  CVSkystoneDetector detector;

  @Override
  public void init() {
    detector = new CVSkystoneDetector(hardwareMap);
    detector.open();
  }

  @Override
  public void loop() {
    Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
    telemetry.addData("position", result.first);
    telemetry.addData("confidence", "%.5f", result.second);
  }
}
