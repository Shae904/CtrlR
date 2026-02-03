package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Red Teleop")
public class RedTeleop extends LinearOpMode {
    public static Robot robot;
    public enum  RunState{
        CYCLEGREEN,
        CYCLEPURPLE,
        INTAKE
    }
    public RunState state;

    public static double cycleTime = Robot.cycleTime;
    public static double outTime = Robot.outTime;
    public static double transferTime = Robot.transferTime;

    public C920PanelsEOCV.C920Pipeline.SlotState[] colors;

    private final ElapsedTime shootTime = new ElapsedTime();
    private int shooting = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.limelight.start();
        state = RunState.INTAKE;
        shootTime.reset();
        telemetry.setMsTransmissionInterval(50);

        while(opModeInInit()){
            robot.setCycle(0);
            robot.updateCycle();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.updateCycle();
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
            telemetry.addData("Timer", shootTime);
            telemetry.addData("Target velocity",out);
            telemetry.addData("Current velocity",robot.launch.getVelocity());
            telemetry.update();

            // Get colors
            colors = robot.pipeline.getSlotStates();

            if (gamepad2.a) {
                state = RunState.INTAKE;
                shooting = 0;
            }
            else if(gamepad2.x){
                state = RunState.CYCLEGREEN;
                if(shooting == 0){
                    shootTime.reset();
                }
            }
            else if(gamepad2.y){
                state = RunState.CYCLEPURPLE;
                if(shooting == 0) {
                    shootTime.reset();
                }
            }
            if(gamepad2.b){
                robot.transferUp();
            }
            else{
                robot.transferDown();
            }
            if(gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
                robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }
            else if(gamepad2.right_trigger > 0.05 || gamepad2.left_trigger > 0.05) {
                robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            }
            else{
                robot.intake.setPower(0);
            }
            switch(state){
                case INTAKE:
                    robot.setCycle(0);
                    robot.transferDown();
                    break;
                case CYCLEGREEN:
                    if(shooting == 0) {
                        for (int i = 0; i < 3; i++) {
                            if (colors[i] == C920PanelsEOCV.C920Pipeline.SlotState.GREEN) {
                                int o = (robot.cpos+i+1) % 3;
                                robot.setCycle(o);
                                telemetry.addData("Green Position:", i);
                                break;
                            }
                        }
                        shooting = 1;
                    }
                    else if(shootTime.seconds() >= cycleTime && shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else if(shootTime.seconds() >= cycleTime + outTime &&  shootTime.seconds() < cycleTime + outTime + transferTime){
                        robot.transferDown();
                    }
                    else if(shootTime.seconds() >= cycleTime + outTime + transferTime){
                        shooting = 0;
                    }
                    break;
                case CYCLEPURPLE:
                    if(shooting == 0) {
                        for (int i = 0; i < 3; i++) {
                            if (colors[i] == C920PanelsEOCV.C920Pipeline.SlotState.PURPLE) {
                                int o = (robot.cpos+i+1) % 3;
                                robot.setCycle(o);
                                telemetry.addData("Purple Position:", i);
                                break;
                            }
                        }
                        shooting = 1;
                    }
                    else if(shootTime.seconds() >= cycleTime && shootTime.seconds() < cycleTime + outTime){
                        robot.transferUp();
                    }
                    else if(shootTime.seconds() >= cycleTime + outTime &&  shootTime.seconds() < cycleTime + outTime + transferTime){
                        robot.transferDown();
                    }
                    else if(shootTime.seconds() >= cycleTime + outTime + transferTime){
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
