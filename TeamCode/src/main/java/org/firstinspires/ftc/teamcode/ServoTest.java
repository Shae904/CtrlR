package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Servo Test")
@Configurable
public class ServoTest extends LinearOpMode {
    Robot robot;
    public static double cyclePos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.cycle1.setPosition(0);
                robot.cycle2.setPosition(0);
            }
            else if(gamepad1.b){
                robot.cycle1.setPosition(cyclePos);
                robot.cycle2.setPosition(cyclePos);
            }
            if(gamepad1.dpad_up){
                robot.cycle1.setPosition(Robot.cyclePos[0]);
                robot.cycle2.setPosition(Robot.cyclePos[0]);
            }
            else if(gamepad1.dpad_left){
                robot.cycle1.setPosition(Robot.cyclePos[1]);
                robot.cycle2.setPosition(Robot.cyclePos[1]);
            }
            else if(gamepad1.dpad_down){
                robot.cycle1.setPosition(Robot.cyclePos[2]);
                robot.cycle2.setPosition(Robot.cyclePos[2]);
            }
            else if(gamepad1.dpad_right){
                robot.cycle1.setPosition(Robot.cyclePos[3]);
                robot.cycle2.setPosition(Robot.cyclePos[3]);
            }
            else if(gamepad1.left_bumper){
                robot.cycle1.setPosition(Robot.cyclePos[4]);
                robot.cycle2.setPosition(Robot.cyclePos[4]);
            }
        }
    }
}