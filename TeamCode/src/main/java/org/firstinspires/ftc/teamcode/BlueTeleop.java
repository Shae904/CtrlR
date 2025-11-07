package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class BlueTeleop extends LinearOpMode{

    Robot robot;
    //LimeLight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        //limelight = robot.getLimelight();
        //limelight.start();
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

            robot.setDriveTrainPower(frontLeftPower,backLeftPower,frontRightPower,backRightPower);

            if(gamepad1.a){
                robot.outtake('b');
            }
            if(gamepad1.y){
                robot.cycleCW();
            }
            if(gamepad1.b){
                robot.cycleCCW();
            }
            if(gamepad1.dpad_down){
                robot.setHood(0);
            }
            else if(gamepad1.dpad_left){
                robot.setHood(0.25);
            }
            else if(gamepad1.dpad_right){
                robot.setHood(0.5);
            }
            else if(gamepad1.dpad_up){
                robot.setHood(0.75);
            }
            else if(gamepad1.left_bumper){
                robot.setHood(1);
            }
            if(gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                robot.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            else{
                robot.setIntakePower(0);
            }
        }
    }
}
