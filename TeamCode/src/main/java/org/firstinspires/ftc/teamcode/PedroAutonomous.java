
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.781, 123.258,Math.toRadians(125)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        follower.followPath(paths.FROMSTARTTOFIRSTSHOOT);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.FIRSTSHOOTTOINTAKEPPG);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.PPGTOSECONDSHOOT);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.SECONDSHOOTTOPGP);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.PGPTOTHIRDSHOOT);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.THIRDSHOOTTOGPP);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.GPPTOLASTSHOOT);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(paths.PARK);
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }


    public static class Paths {
        public PathChain FROMSTARTTOFIRSTSHOOT;
        public PathChain FIRSTSHOOTTOINTAKEPPG;
        public PathChain PPGTOSECONDSHOOT;
        public PathChain SECONDSHOOTTOPGP;
        public PathChain PGPTOTHIRDSHOOT;
        public PathChain THIRDSHOOTTOGPP;
        public PathChain GPPTOLASTSHOOT;
        public PathChain PARK;

        public Paths(Follower follower) {
            FROMSTARTTOFIRSTSHOOT = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.781, 123.258),

                                    new Pose(97.722, 105.854)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(125))
                    .build();

            FIRSTSHOOTTOINTAKEPPG = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.722, 105.854),
                                    new Pose(129.023, 83.245)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(125))
                    .build();

            PPGTOSECONDSHOOT = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.023, 83.245),

                                    new Pose(86.285, 93.272)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(125))

                    .build();

            SECONDSHOOTTOPGP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.285, 93.272),
                                    new Pose(85.646, 51.930),
                                    new Pose(135.642, 58.311)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();

            PGPTOTHIRDSHOOT = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.642, 58.311),
                                    new Pose(116.291, 54.808),
                                    new Pose(81.881, 88.298)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                    .build();

            THIRDSHOOTTOGPP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.881, 88.298),
                                    new Pose(81.603, 18.583),
                                    new Pose(134.358, 37.119)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(6))

                    .build();

            GPPTOLASTSHOOT = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.358, 37.119),

                                    new Pose(87.430, 95.093)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(6), Math.toRadians(37))

                    .build();

            PARK = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.430, 95.093),

                                    new Pose(86.384, 104.424)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }
}
    