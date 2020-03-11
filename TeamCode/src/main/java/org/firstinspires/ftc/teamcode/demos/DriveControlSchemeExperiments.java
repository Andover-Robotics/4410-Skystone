package org.firstinspires.ftc.teamcode.demos;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.StrafingDriveTrain;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Control Scheme Experiments", group="experimental")
public class DriveControlSchemeExperiments extends OpMode {

    private StrafingDriveTrain driveTrain;

    public void init () {
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR"),
                motorBR = hardwareMap.dcMotor.get("motorBR"),
                motorFL = hardwareMap.dcMotor.get("motorFL"),
                motorBL = hardwareMap.dcMotor.get("motorBL");

        driveTrain = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, 50, 300);

    }

    public void loop () {

        // Strafing
        double directionX = 0.0;
        double directionY = 0.0;

        directionX += gamepad1.left_stick_x;
        directionY += gamepad1.left_stick_y;



        if (gamepad1.dpad_up) {
            directionY -= 1;
        }
        if (gamepad1.dpad_down) {
            directionY += 1;
        }

        if (directionY > 1) {
            directionY = 1.0;
        }
        else if (directionY < -1) {
            directionY = -1.0;
        }

        // Rotation ((-) = left, (+) = right)
        double rot = 0.0;

        rot += gamepad1.right_stick_x;

        if (gamepad1.dpad_left) {
            rot -= 1;
        }
        if (gamepad1.dpad_right) {
            rot += 1;
        }

        rot -= gamepad1.left_trigger;
        rot += gamepad1.right_trigger;

        if (rot > 1) {
            rot = 1;
        }
        else if (rot < -1) {
            rot = -1;
        }

        Coordinate coord = Coordinate.fromXY(directionX, -directionY);

        driveTrain.setStrafeRotation(coord, coord.getPolarDistance(), rot);
    }

}
