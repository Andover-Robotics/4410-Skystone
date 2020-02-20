package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "Experimental")
public class LoadSensingDemo extends OpMode {
  private ColorSensor colorSensor;
  private DistanceSensor distanceSensor;

  @Override
  public void init() {
    colorSensor = hardwareMap.colorSensor.get("loadSensor");
    distanceSensor = hardwareMap.get(DistanceSensor.class, "loadSensor");
  }

  @Override
  public void loop() {
    float r = colorSensor.red(), g = colorSensor.green(), b = colorSensor.blue();
    double distance = distanceSensor.getDistance(DistanceUnit.MM);

    telemetry.addData("data", "r=%.4f g=%.4f b=%.4f dist=%.4f mm", r, g, b, distance);
  }
}
