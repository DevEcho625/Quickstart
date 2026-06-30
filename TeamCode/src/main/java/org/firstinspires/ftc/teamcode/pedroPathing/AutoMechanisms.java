package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.FlywheelController;

public class AutoMechanisms {

    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private Servo rampLeft;
    private Servo rampRight;
    private FlywheelController flywheel;

    private ElapsedTime feedTimer = new ElapsedTime();
    private double currentFeedDurationMs = 400;

    public enum ShooterState {
        IDLE,
        SPINNING,
        READY,
        RAMP_DEPLOYING, // NEW STATE: Gives physical servos time to arrive before motors turn on
        FEEDING,
        RECOVERING
    }

    private ShooterState shooterState = ShooterState.IDLE;

    public void init(HardwareMap hardwareMap){
        leftIntake = hardwareMap.get(DcMotorEx.class,"leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class,"rightIntake");
        rampLeft = hardwareMap.get(Servo.class,"rampLeft");
        rampRight = hardwareMap.get(Servo.class,"rampRight");

        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rampLeft.setDirection(Servo.Direction.FORWARD);
        rampRight.setDirection(Servo.Direction.REVERSE);

        flywheel = new FlywheelController();
        flywheel.init(hardwareMap);

        retractRamp();
        stopIntake();
        flywheel.idle();
    }

    public void startIntake(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void stopIntake(){
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void deployRamp(){
        rampLeft.setPosition(0.22);
        rampRight.setPosition(0.22);
    }

    public void retractRamp(){
        rampLeft.setPosition(0.0);
        rampRight.setPosition(0.0);
    }

    public void spoolShooter(){
        flywheel.shoot();
        shooterState = ShooterState.SPINNING;
    }

    public void shoot() {
        shoot(400);
    }

    public void shoot(double durationMs) {
        if (shooterState == ShooterState.READY) {
            this.currentFeedDurationMs = durationMs;

            // FIXED: Start by extending the ramp first, NOT the intake wheels
            deployRamp();
            feedTimer.reset();
            shooterState = ShooterState.RAMP_DEPLOYING;
        }
    }

    public boolean shooterReady(){
        return shooterState == ShooterState.READY;
    }

    public void update() {
        flywheel.update();

        switch (shooterState) {
            case IDLE:
                flywheel.idle();
                break;
            case SPINNING:
                flywheel.shoot();
                if (flywheel.isAtTargetVelocity()) {
                    shooterState = ShooterState.READY;
                }
                break;
            case READY:
                flywheel.shoot();
                break;

            case RAMP_DEPLOYING:
                // FIXED: Wait exactly 150ms for the servos to lift up completely
                // BEFORE feeding elements into the channel
                if (feedTimer.milliseconds() >= 150) {
                    startIntake(1.0); // Now start the intake safely without jams
                    feedTimer.reset(); // Reset timer to track the actual shooting time
                    shooterState = ShooterState.FEEDING;
                }
                break;

            case FEEDING:
                if (feedTimer.milliseconds() >= currentFeedDurationMs) {
                    stopIntake();
                    retractRamp();
                    shooterState = ShooterState.RECOVERING;
                }
                break;
            case RECOVERING:
                flywheel.idle();
                shooterState = ShooterState.IDLE;
                break;
        }
    }

    public void hardStopShooter() {
        flywheel.stop();
        shooterState = ShooterState.IDLE;
    }

    public String getState() {
        return shooterState.toString();
    }
}