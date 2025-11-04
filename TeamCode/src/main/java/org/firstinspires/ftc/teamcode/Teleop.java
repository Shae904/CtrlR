package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode{

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = AngleUnit.DEGREES.toRadians(robot.getHeading());

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Slow mode
            if (gamepad1.right_bumper) {
                frontLeftPower *= 0.4;
                backLeftPower *= 0.4;
                frontRightPower *= 0.4;
                backRightPower *= 0.4;
            }

            robot.fl.setPower(frontLeftPower);
            robot.bl.setPower(backLeftPower);
            robot.fr.setPower(frontRightPower);
            robot.br.setPower(backRightPower);

            if(gamepad1.a){
                robot.shoot();
                telemetry.addData("shoot", "shoot");
            }
            else if(gamepad1.x){
                robot.intake();
                telemetry.addData("intake", "intake");
            }
            else if(gamepad1.y){
                robot.cycleCW();
                telemetry.addData("cycle", "cycle");
            }
            else if(gamepad1.b){
                robot.cycleCCW();
                telemetry.addData("ccw cycle", "ccw cycle");
            }
        }
    }
}
