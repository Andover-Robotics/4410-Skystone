package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

@Autonomous(name = "Reset Encoders", group = "Experimental")
public class ResetEncoders extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
      entry.getValue().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      telemetry.addData(entry.getKey(), entry.getValue().getCurrentPosition());
    }
    telemetry.update();
    waitForStart();
  }
}
