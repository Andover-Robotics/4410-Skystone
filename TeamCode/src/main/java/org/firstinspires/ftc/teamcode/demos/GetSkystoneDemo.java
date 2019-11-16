package org.firstinspires.ftc.teamcode.demos;

import android.util.Pair;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.auto.CVSkystoneDetector;
import org.firstinspires.ftc.teamcode.auto.StonePosition;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Intake;

import java.util.function.Function;

@Autonomous (name = "Get Skystone", group = "Autonomous")
public class GetSkystoneDemo extends LinearOpMode {

    CVSkystoneDetector detector;
    private SampleMecanumDriveBase driveBase;
    private Intake intake;

    public void runOpMode() {
        init_fields();
        waitForStart();

        Pair<StonePosition, Double> result = detector.estimatedSkystonePosition();
        telemetry.addData("position", result.first);
        telemetry.addData("confidence", "%.5f", result.second);

        drive(t -> t.forward(24));
        driveBase.turnSync(-1 * (Math.PI / 2));
        
        switch (result.first) {
            case RIGHT:
                drive(t -> t.forward(0.553).strafeLeft(7.73776)); //  see 2019-11-15 for where these numbers came from
                driveBase.turnSync(-0.65);
                drive(t -> t.back(12));
                break;
            case CENTER:
            case LEFT:
                drive(t -> t.forward(5 + result.first.offsetLeft * 8).strafeLeft(17));
                break;
            default:
                telemetry.addData("Problem", "result.first wasn't initialized");
                telemetry.update();
                while (!isStopRequested());
                return;
        }


        // intake while moving forward
        intake.takeIn(1);
        drive(t -> t.back(1));
        intake.stop();

        // Congrats! You should theoretically have a Skystone.

        detector.close();
    }

    private void init_fields() {
        driveBase = new SampleMecanumDriveREVOptimized(hardwareMap);

        intake = new Intake(hardwareMap.dcMotor.get("intakeLeft"), hardwareMap.dcMotor.get("intakeLeft"));

        detector = new CVSkystoneDetector(hardwareMap);
        detector.open();
    }

    // To prevent literal repetition in runOpMode
    private void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
        driveBase.followTrajectorySync(trajectory.apply(driveBase.trajectoryBuilder()).build());
    }

}
