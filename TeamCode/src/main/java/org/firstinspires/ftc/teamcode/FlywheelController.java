package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class FlywheelController {

    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private VoltageSensor voltageSensor;

    private static final double NOMINAL_VOLTAGE = 13.2;

    // Your tuned PIDF
    private static final double P = 124.8;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 20.6;

    // FIXED: Loosened tolerance window from 40 to 75 ticks/sec to prevent loop stalling
    private static final double VELOCITY_TOLERANCE = 75;

    // Idle and shoot velocities
    private static final double IDLE_VELOCITY = 400;
    private static final double SHOOT_VELOCITY = 1500;

    // Internal state
    private double targetVelocity = 0;
    private long lastPIDUpdate = 0;

    public void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);

        updatePIDF();
        idle();
    }

    private void updatePIDF() {
        double voltage = voltageSensor.getVoltage();
        double compensatedF = F * (NOMINAL_VOLTAGE / voltage);

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, compensatedF);

        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void update() {
        long now = System.currentTimeMillis();

        if (now - lastPIDUpdate > 500) {
            updatePIDF();
            lastPIDUpdate = now;
        }

        // FIXED: Removed the slow manual RAMP_RATE limiting step.
        // Feeds the target velocity straight to the hardware layer for instant acceleration.
        leftShooter.setVelocity(targetVelocity);
        rightShooter.setVelocity(targetVelocity);
    }

    public void shoot() {
        targetVelocity = SHOOT_VELOCITY;
    }

    public void idle() {
        targetVelocity = IDLE_VELOCITY;
    }

    public void stop() {
        targetVelocity = 0;
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }

    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public double getBatteryVoltage() {
        return voltageSensor.getVoltage();
    }

    public boolean isAtTargetVelocity() {
        double averageError = Math.abs(targetVelocity - getAverageVelocity());
        return averageError <= VELOCITY_TOLERANCE;
    }

    public boolean isIdle() {
        return targetVelocity == IDLE_VELOCITY;
    }

    public boolean isStopped() {
        return targetVelocity == 0;
    }

    public String getStatus() {
        if (isStopped()) return "STOPPED";
        if (isIdle()) return "IDLE";
        if (isAtTargetVelocity()) return "READY";
        return "SPINNING";
    }
}