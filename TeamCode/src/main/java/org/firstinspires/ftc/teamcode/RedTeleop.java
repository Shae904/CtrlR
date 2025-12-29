package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Red Teleop")
public class RedTeleop extends LinearOpMode {
    // robot + shooter state
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

        robot.limelight.start();
        robot = new Robot(this);
        state = RunState.INTAKE;
        shootTime.reset();

        waitForStart();

        // first try at reading pattern

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

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
            switch(state){
                case INTAKE:
                    robot.setCycle(0);
                    robot.setLaunch(0);
                    robot.transferDown();
                    robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                    break;
                case SHOOT0:
                    if(shooting == 0){
                        shooting = 1;
                    }
                    robot.outtake('r');
                    if(shootTime.seconds() < cycleTime){
                        robot.setCycle(1);
                    }
                    else if(shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else{
                      robot.transferDown();
                      shooting = 0;
                    }
                    break;
                case SHOOT1:
                    if(shooting == 0){
                        shooting = 1;
                    }
                    robot.outtake('r');
                    if(shootTime.seconds() < cycleTime){
                        robot.setCycle(2);
                    }
                    else if(shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else{
                        robot.transferDown();
                        shooting = 0;
                    }
                    break;
                case SHOOT2:
                    if(shooting == 0){
                        shooting = 1;
                    }
                    robot.outtake('r');
                    if(shootTime.seconds() < cycleTime){
                        robot.setCycle(0);
                    }
                    else if(shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else{
                        robot.transferDown();
                        shooting = 0;
                    }
                    break;
            }
        }
        robot.webcam.stopStreaming();
        robot.webcam.closeCameraDevice();
        robot.limelight.close();
    }

}
