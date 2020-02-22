package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LoadSensor {
  private DistanceSensor sensor;

  public LoadSensor(DistanceSensor sensor) {
    this.sensor = sensor;
  }

  public boolean stonePresent() {
    return sensor.getDistance(DistanceUnit.MM) < 300;
  }
}
