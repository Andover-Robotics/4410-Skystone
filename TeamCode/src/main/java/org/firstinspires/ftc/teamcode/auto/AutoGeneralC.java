package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

public abstract class AutoGeneralC extends SkystoneAuto {

    public void runForCurrentAlliance() {
        driveBase.setPoseEstimate(
                allianceSpecificPoseFromRed(new Pose2d(33, -63, Math.PI / 2)));

        repositionFoundation();

        // Park under the SkyBridge
        driveBase.followTrajectorySync(driveBase.trajectoryBuilder()
                .strafeTo(allianceSpecificPositionFromRed(new Vector2d(0, -59)))
                .build());

        partyUntilItsOver();
    }

}
