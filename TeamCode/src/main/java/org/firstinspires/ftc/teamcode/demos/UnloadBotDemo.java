package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;

@Autonomous(name = "Unload myself", group = "Experimental")
public class UnloadBotDemo extends LinearOpMode {
  @Override
  public void runOpMode() {
    Bot.unloadBot();
    waitForStart();
  }
}
