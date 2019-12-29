package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

public abstract class AutoGeneralC extends SkystoneAuto {

    public void runForCurrentAlliance() {
        driveBase.setPoseEstimate(
                allianceSpecificPoseFromRed(new Pose2d(33, -63, Math.PI / 2)));

        // Drive to the foundation
        driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
                .strafeTo(allianceSpecificPositionFromRed(new Vector2d(51, -30)))
                .build());

        bot.foundationMover.armDown();

        // Pull the foundation to the build site
        driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
                .strafeTo(allianceSpecificPositionFromRed(new Vector2d(51, -70)))
                .splineTo(allianceSpecificPoseFromRed(new Pose2d(58, -64, -0.4)))
                .build());

        bot.foundationMover.armUp();

        // Park under the SkyBridge
        driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
                .strafeTo(allianceSpecificPositionFromRed(new Vector2d(-7, -59)))
                .build());

    }

}
