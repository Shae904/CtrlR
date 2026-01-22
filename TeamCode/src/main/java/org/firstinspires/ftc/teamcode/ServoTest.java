package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Servo Test")
@Configurable
public class ServoTest extends LinearOpMode {
    Robot robot;
    public static double cyclePos = 0;

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
            if(gamepad1.b){
                robot.cycle1.setPosition(cyclePos);
                robot.cycle2.setPosition(cyclePos);
            }
        }
    }
}