package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class MecanumDriveBase extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1.8, 0.2, 0.1);//2.3 0.25 0.55

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

//    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private double turnTarget;
    private double turnStartTime;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    public MecanumDriveBase() {
        super(kV, kA, kStatic, TRACK_WIDTH);

//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    // warning: prefer turnTo in OpModes
    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnTo(heading + angle);
    }

    public void turnTo(double angle) {
        turnTarget = angle;
        // Don't retain the integral sum from the previous turn
        turnController.reset();
        turnController.setTargetPosition(turnTarget);
        turnStartTime = NanoClock.system().seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void turnToSync(double angle) {
        turnTo(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double correction = turnController.update(currentPose.getHeading());

                packet.put("correction", correction);

                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, correction
                )));

                if (Math.abs(currentPose.getHeading() - turnTarget) < Math.toRadians(2.5) ||
                    NanoClock.system().seconds() - turnStartTime > 6) {
                    setDriveSignal(new DriveSignal());
                    mode = Mode.IDLE;
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());

                fieldOverlay.setStroke("#F44336");
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

//        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
