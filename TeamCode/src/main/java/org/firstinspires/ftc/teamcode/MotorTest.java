package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class MotorTest extends LinearOpMode{
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

            if(gamepad1.a){
                robot.setFL();
            }
            else{
                robot.unsetFL();
            }
            if(gamepad1.b){
                robot.setFR();
            }
            else{
                robot.unsetFR();
            }
            if(gamepad1.x){
                robot.setBL();
            }
            else{
                robot.unsetBL();
            }
            if(gamepad1.y){
                robot.setBR();
            }
            else{
                robot.unsetBR();
            }
        }
    }
}