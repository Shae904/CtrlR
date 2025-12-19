package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Blue Teleop")
public class BlueTeleop extends LinearOpMode{

    public static Robot robot;
    int out = 0;
    public ElapsedTime shootTime;
    Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        limelight = robot.getLimelight();
        limelight.start();
        shootTime = new ElapsedTime();
        robot.transfer.setPosition(0.23);
        // START

        // LOOP
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY  = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

            // Slow mode
            if (gamepad1.right_bumper) {
                frontLeftPower *= 0.4;
                backLeftPower *= 0.4;
                frontRightPower *= 0.4;
                backRightPower *= 0.4;
            }

            if (gamepad2.a) {
                if (out == 0) {
                    shootTime.reset();
                }
                robot.outtake('b');
                out = 1;
            } else {
                robot.stopOuttake(out);
                out = 0;
            }
            if (gamepad2.y) {
                robot.setCycle(0);
            }
            if (gamepad2.b) {
                robot.setCycle(1);
            }
            if (gamepad2.x) {
                robot.setCycle(2);
            }
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                robot.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                robot.setIntakePower(gamepad2.right_trigger - gamepad2.left_trigger);
            } else {
                robot.setIntakePower(0);
            }
            telemetry.update();
        }
    }
}
