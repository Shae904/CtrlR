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
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
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

        // panels camera stream using this pipeline as source
        PanelsCameraStream.INSTANCE.startStream(pipeline, 30);

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
        PanelsCameraStream.INSTANCE.stopStream();
    }

    public static class C920Pipeline extends OpenCvPipeline implements CameraStreamSource {
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();        // for generic slider mask / mean hsv
        private final Mat maskGreen = new Mat();   // reused for green masks
        private final Mat maskPurple = new Mat();  // reused for purple masks

        // for triangle masking
        private final Mat slotMask = new Mat();
        private final Mat slotGreen = new Mat();
        private final Mat slotPurple = new Mat();

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

        // slot 0 = triangle (335, 400), (570, 480), (335, 480)
        private final Point[] tri0Points = new Point[] {
                new Point(0, 120),
                new Point(0, 400),
                new Point(120, 400),
                new Point(120, 120)
        };
        private final MatOfPoint tri0Mat = new MatOfPoint(tri0Points);

        // slot 1 & 2 = rectangles (your existing ones)
        // index 0 is null because that slot is triangle-based
        private final Rect[] slotRects = new Rect[] {
                null,
                new Rect(420, 420, 220, 60),  // slot 1
                new Rect(400, 90, 180, 70)   // slot 2
        };

        private final SlotState[] slotStates = new SlotState[] {
                SlotState.EMPTY, SlotState.EMPTY, SlotState.EMPTY
        };

        // tune hsv vals
        private final Scalar greenLower = new Scalar(75, 105, 150);
        private final Scalar greenUpper = new Scalar(93, 255, 255);

        private final Scalar purpleLower = new Scalar(130, 60, 60);
        private final Scalar purpleUpper = new Scalar(160, 255, 255);

        // min colored pixels to detect ball. tune this.
        private final int minPixelsForBall = 400;

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

            // full-frame color masks (for triangle)
            Core.inRange(hsv, greenLower, greenUpper, maskGreen);
            Core.inRange(hsv, purpleLower, purpleUpper, maskPurple);

            // ===== slot 0: triangle =====
            slotMask.create(hsv.rows(), hsv.cols(), CvType.CV_8UC1);
            slotMask.setTo(new Scalar(0));
            Imgproc.fillConvexPoly(slotMask, tri0Mat, new Scalar(255));

            Core.bitwise_and(maskGreen, slotMask, slotGreen);
            Core.bitwise_and(maskPurple, slotMask, slotPurple);

            int greenCount0 = Core.countNonZero(slotGreen);
            int purpleCount0 = Core.countNonZero(slotPurple);

            SlotState state0;
            if (greenCount0 < minPixelsForBall && purpleCount0 < minPixelsForBall) {
                state0 = SlotState.EMPTY;
            } else if (greenCount0 > purpleCount0) {
                state0 = SlotState.GREEN;
            } else {
                state0 = SlotState.PURPLE;
            }
            slotStates[0] = state0;

            // draw triangle
            Scalar triColor;
            switch (state0) {
                case GREEN:
                    triColor = new Scalar(0, 255, 0);
                    break;
                case PURPLE:
                    triColor = new Scalar(255, 0, 255);
                    break;
                default:
                    triColor = new Scalar(255, 255, 255);
            }
            Imgproc.polylines(
                    input,
                    Collections.singletonList(tri0Mat),
                    true,
                    triColor,
                    2
            );

            // ===== slot 1 & 2: rectangles =====
            for (int i = 1; i < slotRects.length; i++) {
                Rect r = slotRects[i];
                if (r == null) {
                    slotStates[i] = SlotState.EMPTY;
                    continue;
                }

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
                if (greenCount > purpleCount && greenCount > minPixelsForBall) {
                    state = SlotState.GREEN;
                } else if (purpleCount > greenCount && purpleCount > minPixelsForBall) {
                    state = SlotState.PURPLE;
                }
                else{
                    state = SlotState.EMPTY;
                }

                slotStates[i] = state;

                // draw rect
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