package org.firstinspires.ftc.teamcode.demos;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutoNav Demo", group = "Experimental")
public class AutoNavDemo extends LinearOpMode {

  // Change these values if necessary
  private static final int TICKS_PER_INCH = (int) (1120 / (4 * Math.PI)), TICKS_PER_360 = 4000;

  @Override
  public void runOpMode() {

    DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
    DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
    DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
    DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

    motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
    motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

    MecanumDrive drivetrain = MecanumDrive.fromOctagonalMotors(
        motorFL, motorFR, motorBL, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);

    waitForStart();

    drivetrain.strafeInches(-5, 10, 1);
    drivetrain.strafeInches(5, 10, 1);
    drivetrain.strafeInches(5, -10, 1);
    drivetrain.strafeInches(-5, -10, 1);
  }
}
