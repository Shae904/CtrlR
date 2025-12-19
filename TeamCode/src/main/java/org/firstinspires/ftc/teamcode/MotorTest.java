package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode{
    Robot robot;
    public static double transfer = 0;
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
                telemetry.addData("Max Velocity",robot.getLaunchVelo());
            }
            if(gamepad1.right_bumper){
                robot.transferUp();
            }
            else{
                robot.transferDown();
            }
            telemetry.update();
        }
    }
}