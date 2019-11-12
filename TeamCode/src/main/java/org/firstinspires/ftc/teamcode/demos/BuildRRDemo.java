package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import kotlin.Unit;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.FoundationMover;

public class BuildRRDemo extends LinearOpMode {
  private SampleMecanumDriveBase driveBase;
  private FoundationMover foundationMover;

  @Override
  public void runOpMode() throws InterruptedException {
    driveBase = new SampleMecanumDriveREVOptimized(hardwareMap);
    foundationMover = new FoundationMover(
        hardwareMap.servo.get("foundationLeft"),
        hardwareMap.servo.get("foundationRight")
    );

    waitForStart();

    driveBase.setPoseEstimate(new Pose2d(39, 60, Math.PI / 2));

    driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
        .splineTo(new Pose2d(60, 30, Math.PI / 2))
        .addMarker(() -> { foundationMover.armDown(); sleep(500); return Unit.INSTANCE; })
        .back(30)
        .addMarker(() -> { foundationMover.armUp(); return Unit.INSTANCE; })
        .strafeRight(60)
        .build());
  }
}
