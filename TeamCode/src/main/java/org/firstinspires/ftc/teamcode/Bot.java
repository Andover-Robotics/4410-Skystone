package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.config.Configuration;
import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.StrafingDriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

  public final StrafingDriveTrain driveTrain;
  public final Intake intake;
  public final ExpansionHubEx hub1, hub2;

  private Configuration mainConfig;

  private Bot(OpMode opMode) {

    initConfig();

    // Hardware Configurations
    driveTrain = MecanumDrive.fromOctagonalMotors(
        motor(opMode, "motorFL"),
        motor(opMode, "motorFR"),
        motor(opMode, "motorBL"),
        motor(opMode, "motorBR"),
        opMode,
        mainConfig.getInt("ticksPerInch"),
        mainConfig.getInt("ticksPer360")
    );
    intake = new Intake(
        motor(opMode, "intakeLeft"),
        motor(opMode, "intakeRight"));

    hub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    hub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

    instance = this;
  }

  // Reduce literal repetition
  private DcMotor motor(OpMode opMode, String deviceName) {
    return opMode.hardwareMap.dcMotor.get(deviceName);
  }

  private void initConfig() {
    try {
      mainConfig = Configuration.fromPropertiesFile("/storage/primary/FIRST/config/main.properties");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
