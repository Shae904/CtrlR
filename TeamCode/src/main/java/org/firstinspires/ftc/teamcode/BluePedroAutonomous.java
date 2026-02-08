
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

@Autonomous(name = "Blue Pathing Autonomous")
@Configurable // Panels
public class BluePedroAutonomous extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.239, 123.258,Math.toRadians(55)));

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
                                    new Pose(21.239, 123.258),

                                    new Pose(46.278, 105.854)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(55))

                    .build();

            FIRSTSHOOTTOINTAKEPPG = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(46.278, 105.854),
                                    new Pose(29.974, 81.868),
                                    new Pose(14.977, 83.245)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(55))

                    .build();

            PPGTOSECONDSHOOT = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.977, 83.245),

                                    new Pose(57.715, 93.272)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(55))

                    .build();

            SECONDSHOOTTOPGP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.715, 93.272),
                                    new Pose(58.354, 51.930),
                                    new Pose(8.358, 58.311)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(55))

                    .build();

            PGPTOTHIRDSHOOT = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.358, 58.311),
                                    new Pose(27.709, 54.808),
                                    new Pose(62.119, 88.298)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            THIRDSHOOTTOGPP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(62.119, 88.298),
                                    new Pose(62.397, 18.583),
                                    new Pose(9.642, 37.119)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(174))

                    .build();

            GPPTOLASTSHOOT = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.642, 37.119),

                                    new Pose(56.570, 95.093)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(174), Math.toRadians(135))

                    .build();

            PARK = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.570, 95.093),

                                    new Pose(57.616, 104.424)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}
    