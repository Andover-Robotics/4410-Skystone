package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Bot;

@Config
@TeleOp(name = "Lift Control Demo", group = "Experimental")
public class LiftControlDemo extends OpMode {
  public static double kP, kI, kD, kF;

  private Bot bot;
  private int setPoint = 0;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    readCoefficients();
  }

  private void readCoefficients() {
    PIDFCoefficients coefficients = bot.slideSystem.getLiftCoefficients();
    kP = coefficients.p;
    kI = coefficients.i;
    kD = coefficients.d;
    kF = coefficients.f;
  }

  public void start() {
    bot.slideSystem.runLiftsToTargetPosition(1);
  }

  @Override
  public void loop() {
    setPoint += gamepad1.left_stick_y * -40;
    telemetry.addData("setpoint", setPoint);
    telemetry.addData("coeffs", "P=%.3f I=%.3f D=%.3f F=%.3f", kP, kI, kD, kF);

    telemetry.addData("press X", "to apply new coeffs");
    if (gamepad1.x) {
      PIDFCoefficients newCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
      bot.slideSystem.setLiftCoefficients(newCoefficients);
    }

    bot.slideSystem.liftRight.setTargetPosition(setPoint);
    bot.slideSystem.liftLeft.setTargetPosition(setPoint);
  }
}
