package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Red Teleop")
public class RedTeleop extends LinearOpMode {
    public static Robot robot;
    public enum  RunState{
        SHOOT0,
        SHOOT1,
        SHOOT2,
        INTAKE
    }
    public RunState state;

    public static double cycleTime = 0.4; // TODO Tune
    public static double outTime = 0.8; // TODO Tune

    private final ElapsedTime shootTime = new ElapsedTime();
    private int shooting = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.limelight.start();
        state = RunState.INTAKE;
        shootTime.reset();
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            /*
            code for if we wanted to use field centric movement
            x = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            y = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            */
            x = x * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

            double out = robot.outtake('r'); // Constantly set flywheel power
            telemetry.addData("Distance to goal",out);
            telemetry.addData("Outtake wheel speed",robot.launch.getVelocity());
            telemetry.update();

            if (gamepad2.a) {
                state = RunState.INTAKE;
                shooting = 0;
            }
            if(gamepad2.x){
                state = RunState.SHOOT0;
                if(shooting == 0){
                    shootTime.reset();
                }
            }
            if(gamepad2.y){
                state = RunState.SHOOT1;
                if(shooting == 0){
                    shootTime.reset();
                }
            }
            if(gamepad2.b){
                state = RunState.SHOOT2;
                if(shooting == 0){
                    shootTime.reset();
                }
            }
            if(gamepad2.right_bumper){
                robot.transferUp();
            }
            else{
                robot.transferDown();
            }
            switch(state){
                case INTAKE:
                    robot.setCycle(0);
                    robot.transferDown();
                    if(gamepad2.right_trigger > 0.05 || gamepad2.left_trigger > 0.05) {
                        robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    }
                    else{
                        robot.intake.setPower(0);
                    }
                    break;
                case SHOOT0:
                    robot.setCycle(1);
                    /*if(shooting == 0){
                        shooting = 1;
                    }
                    if(shootTime.seconds() < cycleTime){
                        robot.setCycle(1);
                    }
                    else if(shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else{
                      robot.transferDown();
                      shooting = 0;
                    }*/
                    break;
                case SHOOT1:
                    robot.setCycle(2);
                    /*if(shooting == 0){
                        shooting = 1;
                    }
                    if(shootTime.seconds() < cycleTime){
                        robot.setCycle(2);
                    }
                    else if(shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else{
                        robot.transferDown();
                        shooting = 0;
                    }*/
                    break;
                case SHOOT2:
                    robot.setCycle(0);
                    /*if(shooting == 0){
                        shooting = 1;
                    }
                    if(shootTime.seconds() < cycleTime){
                        robot.setCycle(0);
                    }
                    else if(shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else{
                        robot.transferDown();
                        shooting = 0;
                    }*/
                    break;
            }
        }
        robot.webcam.stopStreaming();
        robot.webcam.closeCameraDevice();
        robot.limelight.close();
    }

}
