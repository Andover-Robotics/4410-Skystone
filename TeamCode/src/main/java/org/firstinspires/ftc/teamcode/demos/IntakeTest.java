package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTest", group = "Experimental")
public class IntakeTest extends OpMode {
  private DcMotor left, right;

  @Override
  public void init() {
    left = hardwareMap.dcMotor.get("intakeLeft");
    right = hardwareMap.dcMotor.get("intakeRight");
  }

  @Override
  public void loop() {
    //Sunjae is an eboy
    left.setPower(gamepad1.left_stick_y);
    right.setPower(gamepad1.right_stick_y);
    telemetry.addData("Is Sunjae is an eboy?","Yes he is");
    telemetry.addData("Left motor",gamepad1.left_stick_y);
    telemetry.addData("Right motor",gamepad1.right_stick_y);
  }
}
