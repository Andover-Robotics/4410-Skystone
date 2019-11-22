package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.config.Configuration;
import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.StrafingDriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.IOException;

public class Bot {
  private static Bot instance;

  public static Bot getInstance() {
    return instance;
  }

  public static Bot getInstance(OpMode opMode) {
    if (instance == null) {
      instance = new Bot(opMode);
    }
    return instance;
  }

  public final SampleMecanumDriveREVOptimized driveBase;
  public final Intake intake;
  public final ExpansionHubEx hub1, hub2;
  public final BNO055IMU imu;

  private Configuration mainConfig;

  private Bot(OpMode opMode) {
    instance = this;

    initConfig();

    // Hardware Configurations
    driveBase = new SampleMecanumDriveREVOptimized(opMode.hardwareMap);
    imu = driveBase.imu;

    intake = new Intake(
        motor(opMode, "intakeLeft"),
        motor(opMode, "intakeRight"));

    hub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    hub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
  }

  // Reduce literal repetition
  private DcMotor motor(OpMode opMode, String deviceName) {
    return opMode.hardwareMap.dcMotor.get(deviceName);
  }

  private void initConfig() {
    try {
      mainConfig = Configuration.fromPropertiesFile("main.properties");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
