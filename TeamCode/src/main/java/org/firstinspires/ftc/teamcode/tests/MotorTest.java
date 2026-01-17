package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        // START

        // LOOP
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            if(gamepad1.a){robot.setMotor(0,1);}
            else{robot.setMotor(0,0);}

            if(gamepad1.x){robot.setMotor(1,1);}
            else{robot.setMotor(1,0);}

            if(gamepad1.y){robot.setMotor(2,1);}
            else{robot.setMotor(2,0);}

            if(gamepad1.b){robot.setMotor(3,1);}
            else {robot.setMotor(3, 0);}

            if(gamepad1.left_bumper){
                robot.setLaunch(1);
            }
            if(gamepad1.right_bumper){
                robot.transferUp();
            }
            else{
                robot.transferDown();
            }
            telemetry.addData("velocity",robot.launch.getVelocity());
            telemetry.update();
        }
    }
}