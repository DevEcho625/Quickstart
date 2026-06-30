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

@Autonomous(name = "Blue Far Auto", group = "Far")
public class BlueFarAuto extends LinearOpMode {

    private Follower follower;
    private AutoMechanisms mechanisms;

    // --- FAR SIDE BLUE POSES ---
    private final Pose startPose = new Pose(123.5, 123.5, -0.78);

    // ADJUSTED: Brought forward to the absolute legal edge of the far triangle zone
    private final Pose scorePose = new Pose(82.0, 85.5, -0.65);

    // Parked location shifted smoothly behind the line to clear initial perimeter space
    private final Pose parkPose  = new Pose(118.0, 78.0, 0.0);

    private PathChain scorePreload, parkPath;

    public void buildPaths() {
        // Direct tracking down to the absolute edge point
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Push clean back to vacate the area completely
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
                // 1. Spool shooter instantly and trace to the edge line
                instant(() -> mechanisms.spoolShooter()),
                follow(follower, scorePreload),
                waitSeconds(0.25), // Let the chassis settle to maximize trajectory precision

                // 2. Closed-loop RPM confirmation lock
                instant(preloadTimeout::resetTimer),
                waitUntil(() -> mechanisms.shooterReady() || preloadTimeout.getElapsedTimeSeconds() >= 1.5),

                // 3. Smart feader loops out the 3 preloads at matching target velocities
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

                // 4. Back out to secure parking matrix credit
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