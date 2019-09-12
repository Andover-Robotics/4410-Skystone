package org.firstinspires.ftc.teamcode.demos;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.StrafingDriveTrain;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Field Centric Demo", group = "Experimental")
public class FieldCentricDemo extends OpMode {
  private StrafingDriveTrain driveTrain;
  private BNO055IMU imu;

  @Override
  public void init() {
    DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
    DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

    motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
    motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

    driveTrain = MecanumDrive.fromOctagonalMotors(
        hardwareMap.dcMotor.get("motorFL"),
        motorFR,
        hardwareMap.dcMotor.get("motorBL"),
        motorBR,
        this, 250, 2000);
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    initImu();
  }

  private void initImu() {
    BNO055IMU.Parameters params = new BNO055IMU.Parameters();
    params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    params.loggingEnabled = true;
    params.loggingTag = "IMU";
    imu.initialize(params);
  }

  private Coordinate driveVector;

  @Override
  public void loop() {
    driveVector = Coordinate.fromXY(gamepad1.left_stick_x, -gamepad1.left_stick_y)
        .rotate((int) -imu.getAngularOrientation().firstAngle);

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      driveTrain.setRotationPower(gamepad1.right_stick_x);
    }
    driveTrain.setStrafeRotation(driveVector, driveVector.getPolarDistance(), gamepad1.right_stick_x);
  }
}
