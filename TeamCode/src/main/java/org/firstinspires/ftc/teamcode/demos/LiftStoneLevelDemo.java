package org.firstinspires.ftc.teamcode.demos;

import com.andoverrobotics.core.utilities.InputColumnResponder;
import com.andoverrobotics.core.utilities.InputColumnResponderImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Bot;

@Autonomous(name = "Lift Stone Level Demo", group = "Experimental")
public class LiftStoneLevelDemo extends OpMode {
  private Bot bot;
  private InputColumnResponder responder = new InputColumnResponderImpl();
  private int level = 0;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    responder.register(() -> gamepad1.dpad_up, () -> level++)
        .register(() -> gamepad1.dpad_down, () -> level--);
  }

  @Override
  public void loop() {
    telemetry.addLine("controls")
        .addData("Dpad up/down", "level control")
        .addData("gamepad1.x", "run lift to level")
        .addData("bumpers", "software stop");

    if (gamepad1.x) {
      bot.slideSystem.startRunningLiftsToLevel(level);
    }
    if (gamepad1.left_bumper || gamepad1.right_bumper) {
      bot.slideSystem.relaxLift();
    }

    telemetry.addData("level", level);
    telemetry.addData("left current pos", bot.slideSystem.liftLeft.getMotor().getCurrentPosition());
    telemetry.addData("right current pos", bot.slideSystem.liftRight.getMotor().getCurrentPosition());
    telemetry.addData("busy", bot.slideSystem.isLiftRunningToPosition());

    responder.update();
    bot.slideSystem.relayLiftDebugDashboard();
  }
}
