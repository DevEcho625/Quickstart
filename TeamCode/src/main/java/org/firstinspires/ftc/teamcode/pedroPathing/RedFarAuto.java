package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Red Far Auto (Edge Shoot & Park)", group = "Far")
public class RedFarAuto extends LinearOpMode {

    private Follower follower;
    private AutoMechanisms mechanisms;

    // --- FAR SIDE RED POSES (144 - Y Mathematically Reflected) ---
    private final Pose startPose = new Pose(83.5, 8.2, 1.57);

    // ADJUSTED: Re-targeted straight to the lower edge boundary of the triangle
    private final Pose scorePose = new Pose(83.5, 22.2, 0.65);

    private final Pose parkPose  = new Pose(103.25, 22.2, 0.0);

    private PathChain scorePreload, parkPath;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public Command waitSeconds(double seconds) {
        Timer timer = new Timer();
        return sequential(
                instant(timer::resetTimer),
                waitUntil(() -> timer.getElapsedTimeSeconds() >= seconds)
        );
    }

    public Command autoRoutine() {
        Timer preloadTimeout = new Timer();
        Timer preloadFeedTimer = new Timer();

        return sequential(
                // 1. Warm up shooter and trace to the edge boundary
                instant(() -> mechanisms.spoolShooter()),
                follow(follower, scorePreload),
                waitSeconds(0.25),

                // 2. RPM velocity verification lock
                instant(preloadTimeout::resetTimer),
                waitUntil(() -> mechanisms.shooterReady() || preloadTimeout.getElapsedTimeSeconds() >= 1.5),

                // 3. Smart gated feeder loop
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

                // 4. Vacate tile to clear tracking bounds
                parallel(
                        follow(follower, parkPath, true),
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

            telemetry.addData("Shooter State", mechanisms.getState());
            telemetry.update();
        }
    }
}