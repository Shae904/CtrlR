package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

@Configurable
@TeleOp(name = "C920 EasyOpenCV (Panels) + TFLite", group = "Vision")
public class C920PanelsEOCV extends LinearOpMode {

    private OpenCvWebcam webcam;
    private TelemetryManager telemetryM;

    // sliders in panels (HSV stays identical)
    public static int lowerH = 0;
    public static int lowerS = 0;
    public static int lowerV = 0;

    public static int upperH = 180;
    public static int upperS = 255;
    public static int upperV = 255;

    // GREEN (Panels)
    public static int greenLowerH = 75,  greenLowerS = 105, greenLowerV = 80;
    public static int greenUpperH = 93,  greenUpperS = 255, greenUpperV = 255;

    // PURPLE (Panels)
    public static int purpleLowerH = 130, purpleLowerS = 60,  purpleLowerV = 60;
    public static int purpleUpperH = 160, purpleUpperS = 255, purpleUpperV = 255;


    // TFLite controls (separate; does NOT change HSV API)
    public static boolean tfliteEnabled = false;
    public static int tfliteEveryNFrames = 2;
    public static float tfliteMinConf = 0.55f;

    // Put your model here:
    // TeamCode/src/main/assets/slot_classifier_mnv3_small.tflite
    public static String tfliteAssetName = "slot_classifier_mnv3_small.tflite";

    private C920Pipeline pipeline;

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

        pipeline = new C920Pipeline(hardwareMap.appContext);

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
            // HSV telemetry (unchanged)
            telemetryM.debug("Frame count: " + pipeline.getFrameCount());
            telemetryM.debug("Avg H: " + pipeline.getLastH());
            telemetryM.debug("Avg S: " + pipeline.getLastS());
            telemetryM.debug("Avg V: " + pipeline.getLastV());

            // HSV slot states (UNCHANGED API + mapping)
            C920Pipeline.SlotState[] hsvStates = pipeline.getSlotStates();
            telemetryM.debug("Slot 1: " + hsvStates[0]);
            telemetryM.debug("Slot 2: " + hsvStates[1]);
            telemetryM.debug("Slot 3: " + hsvStates[2]);

            // NEW: TFLite states (separate API so nothing breaks)
            C920Pipeline.TFLiteSlotState[] nnStates = pipeline.getTFLiteSlotStates();
            telemetryM.debug("TFLite Slot 1: " + nnStates[0] + " conf=" + pipeline.getTFLiteConf(0));
            telemetryM.debug("TFLite Slot 2: " + nnStates[1] + " conf=" + pipeline.getTFLiteConf(1));
            telemetryM.debug("TFLite Slot 3: " + nnStates[2] + " conf=" + pipeline.getTFLiteConf(2));

            telemetryM.update(telemetry);

            sleep(20);
        }

        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        if (pipeline != null) {
            pipeline.close();
        }

        PanelsCameraStream.INSTANCE.stopStream();
    }

    public static class C920Pipeline extends OpenCvPipeline implements CameraStreamSource {

        // =========================
        // HSV (IDENTICAL to your original)
        // =========================
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat maskGreen = new Mat();
        private final Mat maskPurple = new Mat();

        private final Mat slotMask = new Mat();
        private final Mat slotGreen = new Mat();
        private final Mat slotPurple = new Mat();

        private long frameCount = 0;
        private double lastH = 0, lastS = 0, lastV = 0;

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        public enum SlotState {
            EMPTY,
            GREEN,
            PURPLE
        }

        // slot 0 = triangle (EXACT POINTS)
        private final Point[] tri0Points = new Point[] {
                new Point(0, 30),
                new Point(0, 200),
                new Point(210, 200),
                new Point(110, 30)

        };
        private final MatOfPoint tri0Mat = new MatOfPoint(tri0Points);

        // slot 1 & 2 = rectangles (EXACT RECTS + mapping)
        private final Rect[] slotRects = new Rect[] {
                null,
                new Rect(330, 350, 270, 130),  // slot 1 (maps to slotStates[1])
                new Rect(330, 0, 230, 60)    // slot 2 (maps to slotStates[2])
        };

        private final SlotState[] slotStates = new SlotState[] {
                SlotState.EMPTY, SlotState.EMPTY, SlotState.EMPTY
        };

        private Scalar greenLower() {
            return new Scalar(C920PanelsEOCV.greenLowerH, C920PanelsEOCV.greenLowerS, C920PanelsEOCV.greenLowerV);
        }
        private Scalar greenUpper() {
            return new Scalar(C920PanelsEOCV.greenUpperH, C920PanelsEOCV.greenUpperS, C920PanelsEOCV.greenUpperV);
        }

        private Scalar purpleLower() {
            return new Scalar(C920PanelsEOCV.purpleLowerH, C920PanelsEOCV.purpleLowerS, C920PanelsEOCV.purpleLowerV);
        }
        private Scalar purpleUpper() {
            return new Scalar(C920PanelsEOCV.purpleUpperH, C920PanelsEOCV.purpleUpperS, C920PanelsEOCV.purpleUpperV);
        }


        private final int minPixelsForBall = 1350;

        // =========================
        // TFLite (NEW, but uses SAME slot mapping as HSV)
        // =========================
        public enum TFLiteSlotState {
            EMPTY,
            GREEN,
            PURPLE,
            UNKNOWN
        }

        private final TFLiteSlotState[] tfliteSlotStates = new TFLiteSlotState[] {
                TFLiteSlotState.UNKNOWN, TFLiteSlotState.UNKNOWN, TFLiteSlotState.UNKNOWN
        };
        private final float[] tfliteConf = new float[] {0f, 0f, 0f};

        private Interpreter interpreter = null;

        private static final int NN_W = 96;
        private static final int NN_H = 96;
        private static final int NN_C = 3;

        private final ByteBuffer nnInput =
                ByteBuffer.allocateDirect(1 * NN_W * NN_H * NN_C * 4).order(ByteOrder.nativeOrder());
        private final float[][] nnOutput = new float[1][3];

        // Reuse these mats to avoid per-frame allocs
        private final Mat roiRGBA = new Mat();
        private final Mat roiRGB = new Mat();
        private final Mat roiResizedRGB = new Mat();

        public C920Pipeline(Context appContext) {
            try {
                MappedByteBuffer model = loadModelFromAssets(appContext, tfliteAssetName);
                Interpreter.Options opts = new Interpreter.Options();
                opts.setNumThreads(2);
                interpreter = new Interpreter(model, opts);
            } catch (Throwable t) {
                interpreter = null;
            }
        }

        public void close() {
            try {
                if (interpreter != null) interpreter.close();
            } catch (Exception ignored) {}
            interpreter = null;
        }

        @Override
        public Mat processFrame(Mat input) {
            frameCount++;

            // =========================
            // HSV logic (COPY-PASTE identical)
            // =========================

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(lowerH, lowerS, lowerV);
            Scalar upper = new Scalar(upperH, upperS, upperV);
            Core.inRange(hsv, lower, upper, mask);

            Scalar mean = Core.mean(hsv, mask);
            lastH = mean.val[0];
            lastS = mean.val[1];
            lastV = mean.val[2];

            Core.inRange(hsv, greenLower(), greenUpper(), maskGreen);
            Core.inRange(hsv, purpleLower(), purpleUpper(), maskPurple);


            // ===== slot 0: triangle =====
            slotMask.create(hsv.rows(), hsv.cols(), CvType.CV_8UC1);
            slotMask.setTo(new Scalar(0));
            Imgproc.fillConvexPoly(slotMask, tri0Mat, new Scalar(255));

            Core.bitwise_and(maskGreen, slotMask, slotGreen);
            Core.bitwise_and(maskPurple, slotMask, slotPurple);

            int greenCount0 = Core.countNonZero(slotGreen);
            int purpleCount0 = Core.countNonZero(slotPurple);

            SlotState state0 = SlotState.EMPTY;
            if (greenCount0 > purpleCount0 && greenCount0 > minPixelsForBall) {
                state0 = SlotState.GREEN;
            } else if (purpleCount0 > greenCount0 && purpleCount0 > minPixelsForBall) {
                state0 = SlotState.PURPLE;
            }
            slotStates[0] = state0;

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

                Core.inRange(slotHSV, greenLower(), greenUpper(), maskGreen);
                int greenCount = Core.countNonZero(maskGreen);

                Core.inRange(slotHSV, purpleLower(), purpleUpper(), maskPurple);
                int purpleCount = Core.countNonZero(maskPurple);


                SlotState state = SlotState.EMPTY;
                if (greenCount > purpleCount && greenCount > minPixelsForBall) {
                    state = SlotState.GREEN;
                } else if (purpleCount > greenCount && purpleCount > minPixelsForBall) {
                    state = SlotState.PURPLE;
                }

                slotStates[i] = state;

                Scalar boxColor;
                switch (state) {
                    case GREEN:
                        boxColor = new Scalar(0, 255, 0);
                        break;
                    case PURPLE:
                        boxColor = new Scalar(255, 0, 255);
                        break;
                    default:
                        boxColor = new Scalar(255, 255, 255);
                }
                Imgproc.rectangle(input, bounded, boxColor, 2);

                slotHSV.release();
            }

            // =========================
            // TFLite inference (EXACT SAME slot mapping as HSV)
            //
            // slotStates[] meaning in your original:
            //   slotStates[0] = triangle slot 0 (telemetry labels "Slot 1")
            //   slotStates[1] = rect slotRects[1] (telemetry labels "Slot 2")
            //   slotStates[2] = rect slotRects[2] (telemetry labels "Slot 3")
            //
            // We will match that exactly for tfliteSlotStates[].
            // =========================
            if (tfliteEnabled && interpreter != null &&
                    (tfliteEveryNFrames <= 1 || (frameCount % tfliteEveryNFrames == 0))) {

                // slot 0 (triangle): use bounding rect of the same triangle points
                Rect triBounded = new Rect(
                        0,
                        120,
                        120,
                        220
                );
                triBounded = new Rect(
                        Math.max(0, triBounded.x),
                        Math.max(0, triBounded.y),
                        Math.min(triBounded.width,  Math.max(0, input.cols() - triBounded.x)),
                        Math.min(triBounded.height, Math.max(0, input.rows() - triBounded.y))
                );
                runTFLiteOnROI(input, triBounded, 0);

                // slot 1 & 2 (rectangles): use exact bounded rect logic as HSV
                for (int i = 1; i < slotRects.length; i++) {
                    Rect r = slotRects[i];
                    if (r == null) {
                        tfliteSlotStates[i] = TFLiteSlotState.UNKNOWN;
                        tfliteConf[i] = 0f;
                        continue;
                    }

                    Rect bounded = new Rect(
                            Math.max(0, r.x),
                            Math.max(0, r.y),
                            Math.min(r.width,  Math.max(0, input.cols() - r.x)),
                            Math.min(r.height, Math.max(0, input.rows() - r.y))
                    );

                    runTFLiteOnROI(input, bounded, i);
                }
            }

            // update bitmap for panels (annotated frame) (identical)
            try {
                if (!input.empty()) {
                    Bitmap b = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(input, b);
                    lastFrame.set(b);
                }
            } catch (Exception e) {
                // ignore
            }

            // ---- DEBUG: label slot indices on the stream ----
            try {
                // slot 0 (triangle) label near its top-left
                Imgproc.putText(
                        input, "0",
                        new Point(10, 140),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        new Scalar(255, 255, 255),
                        2
                );

                // slot 1 & 2 labels at the top-left of each bounded rect
                for (int i = 1; i < slotRects.length; i++) {
                    Rect r = slotRects[i];
                    if (r == null) continue;

                    Rect bounded = new Rect(
                            Math.max(0, r.x),
                            Math.max(0, r.y),
                            Math.min(r.width,  Math.max(0, input.cols() - r.x)),
                            Math.min(r.height, Math.max(0, input.rows() - r.y))
                    );
                    if (bounded.width <= 0 || bounded.height <= 0) continue;

                    Imgproc.putText(
                            input,
                            String.valueOf(i),
                            new Point(bounded.x + 5, bounded.y + 25),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            new Scalar(255, 255, 255),
                            2
                    );
                }
            } catch (Exception ignored) { }

            return input;
        }

        // -------------------------
        // TFLite helpers
        // -------------------------
        private void runTFLiteOnROI(Mat fullRGBA, Rect roi, int slotIndex) {
            if (roi.width <= 0 || roi.height <= 0) {
                tfliteSlotStates[slotIndex] = TFLiteSlotState.UNKNOWN;
                tfliteConf[slotIndex] = 0f;
                return;
            }

            try {
                Mat sub = fullRGBA.submat(roi);

                // RGBA -> RGB
                Imgproc.cvtColor(sub, roiRGB, Imgproc.COLOR_RGBA2RGB);

                // resize to 96x96
                Imgproc.resize(roiRGB, roiResizedRGB, new Size(NN_W, NN_H), 0, 0, Imgproc.INTER_AREA);

                // MobileNetV3 preprocess_input: (x/127.5) - 1
                nnInput.rewind();
                int total = (int) (roiResizedRGB.total() * roiResizedRGB.channels());
                byte[] rgb = new byte[total];
                roiResizedRGB.get(0, 0, rgb);

                int idx = 0;
                for (int i = 0; i < NN_W * NN_H; i++) {
                    int r = rgb[idx++] & 0xFF;
                    int g = rgb[idx++] & 0xFF;
                    int b = rgb[idx++] & 0xFF;

                    nnInput.putFloat((r / 127.5f) - 1.0f);
                    nnInput.putFloat((g / 127.5f) - 1.0f);
                    nnInput.putFloat((b / 127.5f) - 1.0f);
                }

                interpreter.run(nnInput, nnOutput);

                float pEmpty = nnOutput[0][0];  // EMPTY
                float pGreen = nnOutput[0][1];  // GREEN
                float pPurple = nnOutput[0][2]; // PURPLE

                int argmax = 0;
                float best = pEmpty;
                if (pGreen > best) { best = pGreen; argmax = 1; }
                if (pPurple > best) { best = pPurple; argmax = 2; }

                tfliteConf[slotIndex] = best;

                if (best < tfliteMinConf) {
                    tfliteSlotStates[slotIndex] = TFLiteSlotState.UNKNOWN;
                } else {
                    tfliteSlotStates[slotIndex] =
                            (argmax == 0) ? TFLiteSlotState.EMPTY :
                                    (argmax == 1) ? TFLiteSlotState.GREEN :
                                            TFLiteSlotState.PURPLE;
                }

                sub.release();
            } catch (Throwable t) {
                tfliteSlotStates[slotIndex] = TFLiteSlotState.UNKNOWN;
                tfliteConf[slotIndex] = 0f;
            }
        }

        private static MappedByteBuffer loadModelFromAssets(Context context, String assetName) throws IOException {
            AssetFileDescriptor afd = context.getAssets().openFd(assetName);
            FileInputStream fis = new FileInputStream(afd.getFileDescriptor());
            FileChannel channel = fis.getChannel();
            long startOffset = afd.getStartOffset();
            long declaredLength = afd.getDeclaredLength();
            return channel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
        }

        // -------------------------
        // Panels CameraStreamSource impl (identical)
        // -------------------------
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
            // nothing here
        }

        // -------------------------
        // HSV getters (UNCHANGED)
        // -------------------------
        public long getFrameCount() { return frameCount; }
        public double getLastH() { return lastH; }
        public double getLastS() { return lastS; }
        public double getLastV() { return lastV; }

        public SlotState[] getSlotStates() { return slotStates; }
        public SlotState getSlotState(int index) { return slotStates[index]; }

        // -------------------------
        // NEW TFLite getters (separate so nothing breaks)
        // -------------------------
        public TFLiteSlotState[] getTFLiteSlotStates() { return tfliteSlotStates; }
        public TFLiteSlotState getTFLiteSlotState(int index) { return tfliteSlotStates[index]; }
        public float getTFLiteConf(int index) { return tfliteConf[index]; }
    }
}