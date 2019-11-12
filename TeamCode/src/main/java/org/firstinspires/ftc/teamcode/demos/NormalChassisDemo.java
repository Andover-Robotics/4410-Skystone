package org.firstinspires.ftc.teamcode.demos;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.StrafingDriveTrain;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Intake;

import java.nio.charset.CoderResult;

@TeleOp(name = "Normal Chassis Demo", group = "Experimental")
public class NormalChassisDemo extends OpMode {
  private StrafingDriveTrain driveTrain;
  private Servo foundMoveLeft, foundMoveRight;
  private Intake intake;
  private BNO055IMU imu;

  @Override
  public void init() {
    DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
    DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");
    DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
    DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");

    motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
    motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

    driveTrain = MecanumDrive.fromOctagonalMotors(
        motorFL,
        motorFR,
        motorBL,
        motorBR,
        this,
        50,
        300
    );

    foundMoveLeft = hardwareMap.servo.get("foundationLeft");
    foundMoveRight = hardwareMap.servo.get("foundationRight");
    foundMoveRight.setDirection(Servo.Direction.REVERSE);

    intake = new Intake(hardwareMap.dcMotor.get("intakeLeft"), hardwareMap.dcMotor.get("intakeRight"));

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

  @Override
  public void loop() {
    driveFieldCentric();

    foundMoveRight.setPosition(0.33 - gamepad1.right_trigger * 0.33);
    foundMoveLeft.setPosition(0.33 - gamepad1.right_trigger * 0.33);

    double intakeSpeed = gamepad1.left_trigger;

    if (gamepad1.left_bumper) {
      intake.takeOut(intakeSpeed);
    } else {
      intake.takeIn(intakeSpeed);
    }
  }

  private void driveFieldCentric() {
    Coordinate driveVector = Coordinate.fromXY(gamepad1.left_stick_x, -gamepad1.left_stick_y)
        .rotate((int) -imu.getAngularOrientation().firstAngle);

    if (Math.abs(gamepad1.right_stick_x) > 0.1 && driveVector.getPolarDistance() < 0.1) {
      driveTrain.setRotationPower(gamepad1.right_stick_x);
    }
    driveTrain.setStrafeRotation(driveVector, driveVector.getPolarDistance(), gamepad1.right_stick_x);
  }
}
