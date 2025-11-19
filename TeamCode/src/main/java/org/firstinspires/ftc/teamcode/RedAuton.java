package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Autonomous")
public class RedAuton extends LinearOpMode {
    public static Robot robot;
    public Limelight3A limelight;
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        limelight = robot.getLimelight();
        limelight.start();
        // START

        // LOOP
        if(opModeIsActive()) {
        }
    }
}
