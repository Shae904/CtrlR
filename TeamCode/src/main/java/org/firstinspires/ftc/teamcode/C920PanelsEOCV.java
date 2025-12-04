package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Configurable
@TeleOp(name = "C920 EasyOpenCV (Panels)", group = "Vision")
public class C920PanelsEOCV extends LinearOpMode {

    private OpenCvWebcam webcam;
    private TelemetryManager telemetryM;

    //HSV values
    public static int lowerH = 0;
    public static int lowerS = 0;
    public static int lowerV = 0;

    public static int upperH = 180;
    public static int upperS = 255;
    public static int upperV = 255;

    private final C920Pipeline pipeline = new C920Pipeline();

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "c920"),
                cameraMonitorViewId
        );

        webcam.setPipeline(pipeline);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        telemetry.addLine("lding");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //streaming when cam opens
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            //pipeline to panels? support live video feed??!
            telemetryM.debug("Frame count: " + pipeline.getFrameCount());
            telemetryM.debug("Avg H: " + pipeline.getLastH());
            telemetryM.debug("Avg S: " + pipeline.getLastS());
            telemetryM.debug("Avg V: " + pipeline.getLastV());
            telemetryM.update(telemetry);

            sleep(20);
        }

        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public static class C920Pipeline extends OpenCvPipeline {
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();

        private long frameCount = 0;
        private double lastH = 0, lastS = 0, lastV = 0;

        @Override
        public Mat processFrame(Mat input) {
            frameCount++;

            // hsv convert
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(lowerH, lowerS, lowerV);
            Scalar upper = new Scalar(upperH, upperS, upperV);

            Core.inRange(hsv, lower, upper, mask);

            //!!TEST!! MEAN HSV
            Scalar mean = Core.mean(hsv, mask);
            lastH = mean.val[0];
            lastS = mean.val[1];
            lastV = mean.val[2];


            return mask;
        }

        public long getFrameCount() { return frameCount; }
        public double getLastH() { return lastH; }
        public double getLastS() { return lastS; }
        public double getLastV() { return lastV; }
    }
}
