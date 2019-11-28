package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JackDemo", group = "Experimental")
public class JackDemo extends OpMode {
  private DcMotor left;
  private DcMotor right;

  @Override
  public void init() {
    left = hardwareMap.dcMotor.get("jackLeft");
    right = hardwareMap.dcMotor.get("jackRight");
  }

  @Override
  public void loop() {
    left.setPower(-gamepad1.left_stick_y * 0.4);
    right.setPower(gamepad1.right_stick_y * 0.4);
  }
}
