package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Transfer Test")
public class TransferTest extends LinearOpMode{
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
            if(gamepad1.y) {
                robot.transferDown();
            }
            if(gamepad1.b){
                robot.transferUp();
            }
        }
    }
}