package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.FoundationMover;

import java.util.function.Function;

@Autonomous(name = "BuildStart Road Runner Demo", group = "Experimental")
public class BuildRRDemo extends LinearOpMode {
  private SampleMecanumDriveBase driveBase;
  private FoundationMover foundationMover;

  @Override
  public void runOpMode() {
    driveBase = new SampleMecanumDriveREVOptimized(hardwareMap);
    foundationMover = new FoundationMover(
        hardwareMap.servo.get("foundationLeft"),
        hardwareMap.servo.get("foundationRight")
    );

    waitForStart();

    driveBase.setPoseEstimate(new Pose2d(39, 60, -Math.PI / 2));


    drive(t -> t.strafeTo(new Vector2d(60, 30)));

    foundationMover.armDown();
    sleep(700);

    drive(t -> t.back(40));

    foundationMover.armUp();
    sleep(800);

    drive(t -> t.strafeRight(60));
  }

  // To prevent literal repetition in runOpMode
  private void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
    driveBase.followTrajectorySync(trajectory.apply(driveBase.trajectoryBuilder()).build());
  }

}
