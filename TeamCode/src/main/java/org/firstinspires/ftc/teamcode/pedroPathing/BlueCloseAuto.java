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

@Autonomous(name = "Blue Auto 12-Ball", group = "Main")
public class BlueCloseAuto extends LinearOpMode {

    private Follower follower;
    private AutoMechanisms mechanisms;

    // --- POSES ---
    private final Pose startPose = new Pose(21, 123.5, 2.37);
    private final Pose scorePose = new Pose(50.66, 88.85, 2.3);
    private final Pose pickup1Pose = new Pose(17, 85, 3.14);
    private final Pose pickup2Pose = new Pose(17.45, 57.388, 3.12);
    private final Pose pickup3Pose = new Pose(14.63, 36.19, 3.12);
    private final Pose endPose = new Pose(21.5, 71.67, 0);

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public void buildPaths() {
        scorePreload = follower.pathBuilder().addPath(new BezierLine(startPose, scorePose)).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()).build();

        // Optimized wide Bezier exit vectors to preserve chassis momentum and prevent wheel slippage
        grabPickup1 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(45, 95), pickup1Pose)).setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading()).build();
        scorePickup1 = follower.pathBuilder().addPath(new BezierLine(pickup1Pose, scorePose)).setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading()).build();

        grabPickup2 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(55, 65), pickup2Pose)).setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading()).build();
        scorePickup2 = follower.pathBuilder().addPath(new BezierCurve(pickup2Pose, new Pose(55, 65), scorePose)).setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading()).build();

        grabPickup3 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(60, 40), pickup3Pose)).setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading()).build();
        scorePickup3 = follower.pathBuilder().addPath(new BezierCurve(pickup3Pose, new Pose(60, 40), scorePose)).setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading()).build();

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
        Timer dynamicFeedTimer = new Timer();

        return sequential(
                // Maximize intake speed to pull pieces inside cleanly before chassis plows over them
                parallel(
                        instant(() -> mechanisms.startIntake(1.0)),
                        follow(follower, grabPath, true)
                ),
                waitSeconds(0.3),
                instant(() -> mechanisms.stopIntake()),

                parallel(
                        instant(() -> mechanisms.spoolShooter()),
                        follow(follower, scorePath, true)
                ),
                waitSeconds(0.2), // Structural settling time to stop chassis oscillation

                // Velocity Verification Gate
                instant(safetyTimeout::resetTimer),
                waitUntil(() -> mechanisms.shooterReady() || safetyTimeout.getElapsedTimeSeconds() >= 1.2),

                // SMART CYCLE FEEDING: Dynamically gates intake power based on real-time RPM checks
                instant(() -> {
                    mechanisms.deployRamp();
                    dynamicFeedTimer.resetTimer();
                }),

                // Keep feeding active for 800ms total, but evaluate velocity recovery constantly
                waitUntil(() -> {
                    if (dynamicFeedTimer.getElapsedTimeSeconds() >= 0.8) {
                        mechanisms.stopIntake();
                        mechanisms.retractRamp();
                        return true;
                    }
                    if (mechanisms.shooterReady()) {
                        mechanisms.startIntake(1.0); // Velocity is nominal -> Push ball forward
                    } else {
                        mechanisms.stopIntake();     // Flywheel dipped -> Stall intake to recover energy
                    }
                    return false;
                }),
                waitSeconds(0.2)
        );
    }

    public Command autoRoutine() {
        Timer preloadTimeout = new Timer();
        Timer preloadFeedTimer = new Timer();

        return sequential(
                // --- PRELOAD PHASE ---
                instant(() -> mechanisms.spoolShooter()),
                follow(follower, scorePreload),
                waitSeconds(0.2),

                // Lock initial preload shot consistency
                instant(preloadTimeout::resetTimer),
                waitUntil(() -> mechanisms.shooterReady() || preloadTimeout.getElapsedTimeSeconds() >= 1.5),

                // SMART PRELOAD FEEDING: Ensures all baseline preload elements fire at exact matching RPMs
                instant(() -> {
                    mechanisms.deployRamp();
                    preloadFeedTimer.resetTimer();
                }),
                waitUntil(() -> {
                    if (preloadFeedTimer.getElapsedTimeSeconds() >= 1.2) {
                        mechanisms.stopIntake();
                        mechanisms.retractRamp();
                        return true;
                    }
                    if (mechanisms.shooterReady()) {
                        mechanisms.startIntake(1.0);
                    } else {
                        mechanisms.stopIntake();
                    }
                    return false;
                }),
                waitSeconds(0.2),

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