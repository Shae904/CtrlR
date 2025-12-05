package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.C920PanelsEOCV;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Red Teleop")
public class RedTeleop extends LinearOpMode {

    //pipelinez
    private OpenCvWebcam webcam;
    private C920PanelsEOCV.C920Pipeline pipeline;
    private C920PanelsEOCV.C920Pipeline.SlotState[] prevColors = new C920PanelsEOCV.C920Pipeline.SlotState[3];
    private int[] frameCount = new int[3];
    private boolean autoFire = false;

    public static Robot robot;
    int out = 0;
    public ElapsedTime shootTime;
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "c920"), cameraMonitorViewId);
        pipeline = new C920PanelsEOCV.C920Pipeline();
        webcam.setPipeline(pipeline);
        // acc view is optional
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        telemetry.addLine("Initializing camera...");
        telemetry.update();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        // init frame count
        for (int i = 0; i < 3; i++) {
            prevColors[i] = C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
            frameCount[i] = 0;
        }

        waitForStart();

        limelight = robot.getLimelight();
        limelight.start();
        shootTime = new ElapsedTime();
        robot.transfer.setPosition(0.23);
        // starttele

        // Main loop
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower  = (rotY - rotX + rx) / denominator;
            double frontRightPower= (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

            //intake
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                robot.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                robot.setIntakePower(gamepad2.right_trigger - gamepad2.left_trigger);
            } else {
                robot.setIntakePower(0);
            }

            // shooting control (manual and automatic)
            if (gamepad2.a) {
                // driver manual shoot command overrides auto
                autoFire = false;
                if (out == 0) {
                    shootTime.reset();
                }
                robot.outtake('r', shootTime.seconds());
                out = 1;
            } else if (autoFire) {
                // continue an ongoing automatic outtake sequence
                robot.outtake('r', shootTime.seconds());
                out = 1;
                // if the shot cycle time has elapsed, stop shooting
                if (shootTime.seconds() > 2 * Robot.cycleTime + Robot.outTime) {
                    robot.stopOuttake(0);
                    out = 0;
                    autoFire = false;
                }
            } else {
                // no shoot button pressed and no auto sequence active
                if (out == 1) {
                    // manual shot was completed in the previous loop, stop the outtake and advance indexer
                    robot.stopOuttake(1);
                    out = 0;
                }
                // read vision pipeline to possibly trigger an automatic shot
                C920PanelsEOCV.C920Pipeline.SlotState[] states = pipeline.getSlotStates();
                for (int i = 0; i < 3; i++) {
                    C920PanelsEOCV.C920Pipeline.SlotState current = states[i];
                    if (current != C920PanelsEOCV.C920Pipeline.SlotState.EMPTY) {
                        // increase count if same color seen again, or reset??
                        if (current == prevColors[i]) {
                            frameCount[i]++;
                        } else {
                            frameCount[i] = 1;
                            prevColors[i] = current;
                        }
                    } else {
                        // Slot is empty or no target, reset tracking
                        frameCount[i] = 0;
                        prevColors[i] = C920PanelsEOCV.C920Pipeline.SlotState.EMPTY;
                    }
                    // If color detected consistently for 3+ frames, trigger outtake for that slot
                    if (frameCount[i] >= 3 && current != C920PanelsEOCV.C920Pipeline.SlotState.EMPTY) {
                        // Confirmed a GREEN/PURPLE ball in chamber i
                        robot.setCycle(i);             // move indexer to that chamber
                        shootTime.reset();            // reset timer for the shot cycle
                        robot.outtake('r', shootTime.seconds());  // start shooting (shooter spins up and feeds ball)
                        out = 1;
                        autoFire = true;             // mark that an auto shot is in progress
                        break;                       // only handle one ball at a time
                    }
                }
            }

            // manual indexing
            if (gamepad2.y) {
                robot.setCycle(0);
            }
            if (gamepad2.b) {
                robot.setCycle(1);
            }
            if (gamepad2.x) {
                robot.setCycle(2);
            }

            // tels
            telemetry.update();
        }

        // Cleanup: stop vision pipeline on exit
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
