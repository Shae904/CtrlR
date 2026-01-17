package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.Robot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Cycle Test")
@Configurable
public class CycleTest extends LinearOpMode{
    Robot robot;
    public static double cyclePos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {
            if(gamepad1.a){
                robot.setCyclePosition(cyclePos);
            }
            if(gamepad1.x){
                robot.setCycle(0);
            }
            if(gamepad1.y){
                robot.setCycle(1);
            }
            if(gamepad1.b){
                robot.setCycle(2);
            }
        }
    }
}