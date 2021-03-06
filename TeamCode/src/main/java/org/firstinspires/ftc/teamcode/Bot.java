package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.andoverrobotics.core.config.Configuration;
import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.StrafingDriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

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

  public static void unloadBot() {
    instance = null;
    System.gc();
  }

  public final StrafingDriveTrain driveTrain;
  public final Intake intake;
  public final FoundationMover foundationMover;
  public final SlideSystem slideSystem;
  public final ExpansionHubEx hub1, hub2;
  public final SideClaw sideClaw;
  public final LoadSensor loadSensor;
  public static BNO055IMU imu;

  public Configuration mainConfig;

  private Bot(OpMode opMode) {

    initConfig();

    DcMotor motorFR = opMode.hardwareMap.dcMotor.get("motorFR");
    DcMotor motorBR = opMode.hardwareMap.dcMotor.get("motorBR");
    DcMotor motorFL = opMode.hardwareMap.dcMotor.get("motorFL");
    DcMotor motorBL = opMode.hardwareMap.dcMotor.get("motorBL");

    motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
    motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

    driveTrain = MecanumDrive.fromOctagonalMotors(
        motorFL,
        motorFR,
        motorBL,
        motorBR,
        opMode,
        50,
        300
    );
    driveTrain.setDefaultDrivePower(1.0);

    intake = new Intake(
        motor(opMode, "intakeLeft"),
        motor(opMode, "intakeRight"));

    foundationMover = new FoundationMover(
        opMode.hardwareMap.servo.get("foundationLeft"),
        opMode.hardwareMap.servo.get("foundationRight"));

    slideSystem = new SlideSystem(
        opMode.hardwareMap.dcMotor.get("liftL"),
        opMode.hardwareMap.dcMotor.get("liftR"),
        opMode.hardwareMap.servo.get("clamp"),
        opMode.hardwareMap.servo.get("fourBar"));

    hub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    hub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

    sideClaw = new SideClaw(
        opMode.hardwareMap.servo.get("sideClawArm"),
        opMode.hardwareMap.servo.get("sideClawClamp"));

    loadSensor = new LoadSensor(opMode.hardwareMap.get(DistanceSensor.class, "loadSensor"));

    instance = this;
    initImu(opMode);
  }

  public void initImu(OpMode opMode) {
    if (imu == null) {
      imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
      imu.initialize(parameters);
      BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
      Log.v("4410-2020 IMU", "Initialized");
    }
  }

  // Reduce literal repetition
  private DcMotor motor(OpMode opMode, String deviceName) {
    return opMode.hardwareMap.dcMotor.get(deviceName);
  }

  private void initConfig() {
    try {
      mainConfig = Configuration.fromPropertiesFile("mainConfig.properties");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
