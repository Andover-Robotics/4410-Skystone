package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;

@Autonomous(group = "Experimental")
public class SideClawDemo extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Bot bot = Bot.getInstance(this);
    waitForStart();

    bot.sideClaw.armUp();
    sleep(1000);
    bot.sideClaw.release();
    sleep(1000);
    bot.sideClaw.armDown();
    sleep(1000);
    bot.sideClaw.clamp();
  }
}
