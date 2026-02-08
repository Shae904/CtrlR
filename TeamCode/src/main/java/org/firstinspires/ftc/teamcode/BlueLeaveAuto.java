package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Leave")
public class BlueLeaveAuto extends LinearOpMode {

    // ===== robot + pedro =====
    public static Robot robot;
    public Follower follower;
    // ===== paths (your coords) =====
    public static class Paths {
        public PathChain PARK;
        public Paths(Follower follower) {
            PARK = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57, 9),
                            new Pose(56, 30)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }

    private Paths paths;

    // ===== state machine =====
    private enum State{

        PARK,
        STOP
    }

    private State state = State.PARK;
    private final ElapsedTime opTimer = new ElapsedTime();

    /**
     * Scale current drivetrain motor powers (used to cap speed on intake legs).
     */

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        follower = Constants.createFollower(hardwareMap);

        paths = new Paths(follower);

        waitForStart();
        robot.intake.setPower(0);
        opTimer.reset();

        follower.setStartingPose(new Pose(87, 9,Math.toRadians(90)));

        state = State.PARK;

        while (opModeIsActive() && state != State.STOP) {
            // safety end
            if (opTimer.seconds() > 29.5) state = State.STOP;

            switch (state) {
                case PARK:
                    follower.followPath(paths.PARK, true);
                    while (opModeIsActive() && follower.isBusy()) {
                        follower.update();
                        robot.updateCycle();
                        telemetry.addData("state", state);
                        telemetry.update();
                    }
                    state = State.STOP;
                    break;

                default:
                    state = State.STOP;
                    break;
            }
        }
    }
}
