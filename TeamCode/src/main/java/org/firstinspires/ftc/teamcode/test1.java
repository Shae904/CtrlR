package org.firstinspires.ftc.teamcode;

import android.widget.ToggleButton;

import com.qualcomm.robotcore.eventloop.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "little Bertha")
public class test1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "fl");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "fr");
        DcMotor BackLeft = hardwareMap.get(DcMotor.class, "bl");
        DcMotor BackRight = hardwareMap.get(DcMotor.class, "br");



        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// Initiating DC motors for drivetrain complete




        waitForStart();

        while (opModeIsActive()){


            /// Gamepad 1
            /// Driving Meccunam
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.7; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            FrontLeft.setPower(frontLeftPower);
            FrontRight.setPower(frontRightPower);
            BackLeft.setPower(-(backLeftPower));
            BackRight.setPower(-(backRightPower));

//            /// Gamepad 2
//
//
//            telemetry.addData("Shoulder", Shoulder.getCurrentPosition());
//            telemetry.addData("Shoulder_Support", Shoulder_Support.getCurrentPosition());
//            telemetry.addData("Slider", Slider.getCurrentPosition());
//            telemetry.addData("Wrist1 Power", wrist1.getPower());
//            telemetry.addData("Wrist2 Power", wrist2.getPower());
            telemetry.addData("Steering BF", -gamepad1.left_stick_y );
            telemetry.addData("Steering LR", gamepad1.left_stick_x );
            telemetry.update();

        }
    }
}
