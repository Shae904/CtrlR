package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import com.bylazar.camerastream.PanelsCameraStream;
import java.util.concurrent.atomic.AtomicReference;


@Configurable
@TeleOp(name = "C920 EasyOpenCV (Panels)", group = "Vision")
public class C920PanelsEOCV extends LinearOpMode {

    private OpenCvWebcam webcam;
    private TelemetryManager telemetryM;

    // sliders in panels
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

        // start panels camera stream using this pipeline as source
        PanelsCameraStream.INSTANCE.startStream(pipeline);

        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        telemetry.addLine("lding");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // streaming when cam opens
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
            // telemetry to panels
            telemetryM.debug("Frame count: " + pipeline.getFrameCount());
            telemetryM.debug("Avg H: " + pipeline.getLastH());
            telemetryM.debug("Avg S: " + pipeline.getLastS());
            telemetryM.debug("Avg V: " + pipeline.getLastV());

            // slot states
            C920Pipeline.SlotState[] states = pipeline.getSlotStates();
            telemetryM.debug("Slot 1: " + states[0]);
            telemetryM.debug("Slot 2: " + states[1]);
            telemetryM.debug("Slot 3: " + states[2]);

            telemetryM.update(telemetry);

            sleep(20);
        }

        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        // stop panels camera stream
        PanelsCameraStream.INSTANCE.stopStream(pipeline);
    }

    public static class C920Pipeline extends OpenCvPipeline implements CameraStreamSource {
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();        // for generic slider mask / mean hsv
        private final Mat maskGreen = new Mat();   // per-slot green mask
        private final Mat maskPurple = new Mat();  // per-slot purple mask

        private long frameCount = 0;
        private double lastH = 0, lastS = 0, lastV = 0;

        // latest frame for panels camerastream
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        public enum SlotState {
            EMPTY,
            GREEN,
            PURPLE
        }

        // TODO: change these rects to match where the 3 slots actually are in the image
        private final Rect[] slotRects = new Rect[] {
                new Rect(50, 200, 80, 80),   // slot 1
                new Rect(170, 200, 80, 80),  // slot 2
                new Rect(290, 200, 80, 80)   // slot 3
        };

        private final SlotState[] slotStates = new SlotState[] {
                SlotState.EMPTY, SlotState.EMPTY, SlotState.EMPTY
        };

        // tune hsv vals
        private final Scalar greenLower = new Scalar(40, 70, 70);
        private final Scalar greenUpper = new Scalar(90, 255, 255);

        private final Scalar purpleLower = new Scalar(130, 60, 60);
        private final Scalar purpleUpper = new Scalar(160, 255, 255);

        // min colored pixels to detect ball. tune this.
        private final int minPixelsForBall = 300;

        @Override
        public Mat processFrame(Mat input) {
            frameCount++;

            // convert to hsv
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            // slider mask
            Scalar lower = new Scalar(lowerH, lowerS, lowerV);
            Scalar upper = new Scalar(upperH, upperS, upperV);
            Core.inRange(hsv, lower, upper, mask);

            Scalar mean = Core.mean(hsv, mask);
            lastH = mean.val[0];
            lastS = mean.val[1];
            lastV = mean.val[2];

            // slot classification
            for (int i = 0; i < slotRects.length; i++) {
                Rect r = slotRects[i];

                // clamp rect to image bounds
                Rect bounded = new Rect(
                        Math.max(0, r.x),
                        Math.max(0, r.y),
                        Math.min(r.width,  Math.max(0, hsv.cols() - r.x)),
                        Math.min(r.height, Math.max(0, hsv.rows() - r.y))
                );

                if (bounded.width <= 0 || bounded.height <= 0) {
                    slotStates[i] = SlotState.EMPTY;
                    continue;
                }

                Mat slotHSV = hsv.submat(bounded);

                // green mask
                Core.inRange(slotHSV, greenLower, greenUpper, maskGreen);
                int greenCount = Core.countNonZero(maskGreen);

                // purple mask
                Core.inRange(slotHSV, purpleLower, purpleUpper, maskPurple);
                int purpleCount = Core.countNonZero(maskPurple);

                SlotState state;
                if (greenCount < minPixelsForBall && purpleCount < minPixelsForBall) {
                    state = SlotState.EMPTY;
                } else if (greenCount > purpleCount) {
                    state = SlotState.GREEN;
                } else {
                    state = SlotState.PURPLE;
                }

                slotStates[i] = state;

                // draw a rectangle showing classification on the output frame
                Scalar boxColor;
                switch (state) {
                    case GREEN:
                        boxColor = new Scalar(0, 255, 0);       // green box
                        break;
                    case PURPLE:
                        boxColor = new Scalar(255, 0, 255);     // magenta box
                        break;
                    default:
                        boxColor = new Scalar(255, 255, 255);   // white for empty
                }
                Imgproc.rectangle(input, bounded, boxColor, 2);

                slotHSV.release();
            }

            // update bitmap for panels (annotated frame)
            try {
                if (!input.empty()) {
                    Bitmap b = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(input, b);
                    lastFrame.set(b);
                }
            } catch (Exception e) {
                // ignore, don't crash on weird frame
            }

            // return annotated rgb image so ds preview + panels see boxes
            return input;
        }

        // camerastreamsource impl for panels

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        @Override
        public void onDrawFrame(Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {
            // nothing here, overlays are done in processFrame
        }

        public long getFrameCount() { return frameCount; }
        public double getLastH() { return lastH; }
        public double getLastS() { return lastS; }
        public double getLastV() { return lastV; }

        public SlotState[] getSlotStates() { return slotStates; }
        public SlotState getSlotState(int index) { return slotStates[index]; }
    }
}
