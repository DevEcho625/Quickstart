package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.util.Timer;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.Scheduler.*;

@Autonomous(name = "Blue Auto 12-Ball ", group = "Main")
public class BlueAuto extends LinearOpMode {

    private Follower follower;
    private AutoMechanisms mechanisms;

    // --- POSES ---
    private final Pose startPose = new Pose(21, 123.5, 2.37);
    private final Pose scorePose = new Pose(48.66, 86.85, 2.21);
    private final Pose pickup1Pose = new Pose(17, 85, 3.14);
    private final Pose pickup2Pose = new Pose(17.45, 57.388, 3.12);
    private final Pose pickup3Pose = new Pose(14.63, 36.19, 3.12); // Kept further out for row 3 alignment
    private final Pose endPose = new Pose(21.5, 71.67, 0);

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public void buildPaths() {
        scorePreload = follower.pathBuilder().addPath(new BezierLine(startPose, scorePose)).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()).build();
        grabPickup1 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(54, 70), pickup1Pose)).setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading()).build();
        scorePickup1 = follower.pathBuilder().addPath(new BezierLine(pickup1Pose, scorePose)).setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading()).build();
        grabPickup2 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(60, 54), pickup2Pose)).setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading()).build();
        scorePickup2 = follower.pathBuilder().addPath(new BezierCurve(pickup2Pose, new Pose(60, 54), scorePose)).setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading()).build();
        grabPickup3 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(60, 30), pickup3Pose)).setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading()).build();
        scorePickup3 = follower.pathBuilder().addPath(new BezierCurve(pickup3Pose, new Pose(60, 30), scorePose)).setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading()).build();
        leave = follower.pathBuilder().addPath(new BezierLine(scorePose, endPose)).setConstantHeadingInterpolation(scorePose.getHeading()).build();
    }

    public Command waitSeconds(double seconds) {
        Timer timer = new Timer();
        return sequential(
                instant(timer::resetTimer),
                waitUntil(() -> timer.getElapsedTimeSeconds() >= seconds)
        );
    }

    public Command sampleCycle(PathChain grabPath, PathChain scorePath) {
        Timer safetyTimeout = new Timer();

        return sequential(
                parallel(
                        instant(() -> mechanisms.startIntake(0.7)),
                        follow(follower, grabPath, true)
                ),
                waitSeconds(0.3),
                instant(() -> mechanisms.stopIntake()),

                parallel(
                        instant(() -> mechanisms.spoolShooter()),
                        follow(follower, scorePath, true)
                ),
                waitSeconds(0.15),

                instant(safetyTimeout::resetTimer),
                waitUntil(() -> mechanisms.shooterReady() || safetyTimeout.getElapsedTimeSeconds() >= 1.2),

// Keep 800ms for regular dual-ball collection cycles
                instant(() -> mechanisms.shoot(800)),
                waitSeconds(1.0)
        );
    }

    public Command autoRoutine() {
        Timer preloadTimeout = new Timer();
        return sequential(
// --- PRELOAD PHASE ---
                instant(() -> mechanisms.spoolShooter()),
                follow(follower, scorePreload),
                waitSeconds(0.15),

                instant(preloadTimeout::resetTimer),
                waitUntil(() -> mechanisms.shooterReady() || preloadTimeout.getElapsedTimeSeconds() >= 1.5),

// FIXED: Keeping the feed time long (1000ms) but it will now be unjammed due to the 150ms servo delay
                instant(() -> mechanisms.shoot(1000)),
                waitSeconds(1.2),

// --- CYCLING PHASE ---
                sampleCycle(grabPickup1, scorePickup1),
                sampleCycle(grabPickup2, scorePickup2),
                sampleCycle(grabPickup3, scorePickup3),

// --- PARK PHASE ---
                parallel(
                        follow(follower, leave, true),
                        instant(() -> mechanisms.hardStopShooter())
                )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms();
        mechanisms.init(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            mechanisms.update();
            Scheduler.execute();

            telemetry.addData("Shooter FSM State", mechanisms.getState());
            telemetry.addData("Follower Moving", follower.isBusy());
            telemetry.update();
        }
    }
}
