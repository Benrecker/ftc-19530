package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Elevator {
    public static Elevator.Params PARAMS = new Elevator.Params();
//    private FtcDashboard dash = FtcDashboard.getInstance();

    private final DcMotorEx elevator_left;
    private final DcMotorEx elevator_right;

    private final DcMotorEx wench;

    ServoImplEx servo_hook_left;
    ServoImplEx servo_hook_right;

    private final PIDFController pid_left;
    private final PIDFController pid_right;

    private final Timing.Timer reset_encoder_timer;

    public Elevator(HardwareMap hardwareMap) {

        elevator_left = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevator_right = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        wench = hardwareMap.get(DcMotorEx.class, "motorWench");

        servo_hook_left = hardwareMap.get(ServoImplEx.class, "servoHookLeft");
        servo_hook_right = hardwareMap.get(ServoImplEx.class, "servoHookRight");

        // Set idle behavior
        elevator_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wench.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motor direction
        elevator_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_hook_left.setDirection(Servo.Direction.REVERSE);

        elevator_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wench.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wench.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid_left = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        pid_right = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);

        pid_left.setP(PARAMS.kP);
        pid_left.setI(PARAMS.kI);
        pid_left.setD(PARAMS.kD);

        pid_right.setP(PARAMS.kP);
        pid_right.setI(PARAMS.kI);
        pid_right.setD(PARAMS.kD);

        reset_encoder_timer = new Timing.Timer(3);

        servo_hook_left.setPwmEnable();
        servo_hook_right.setPwmEnable();

        servo_hook_left.setPosition(PARAMS.hook_target);
        servo_hook_right.setPosition(PARAMS.hook_target);
    }

    /////////////////////////////////////////////////////////////

    public void setHeight(double height) {
        PARAMS.target_height = Math.min(Math.max(PARAMS.encoderMinimum, height), PARAMS.encoderMaximum);
    }

    public boolean atTarget() {
        return Math.abs(((elevator_left.getCurrentPosition() + elevator_right.getCurrentPosition()) / 2.0) - PARAMS.target_height) < 80.0;
    }
    public boolean atTarget(double error) {
        return Math.abs(((elevator_left.getCurrentPosition() + elevator_right.getCurrentPosition()) / 2.0) - PARAMS.target_height) < error;
    }

    public double getCurrentHeight() {
        return (elevator_left.getCurrentPosition() + elevator_right.getCurrentPosition()) / 2.0;
    }

    public void setAutoReset(boolean enabled) {
        PARAMS.elevator_enable_auto_reset = enabled;
    }


    /////////////////////////////////////////////////////////////
    public void setHookPercent(double percent) {
        PARAMS.hook_target = clamp(map(percent, 0.0, 1.0, PARAMS.hook_outside, PARAMS.hook_inside), PARAMS.hook_outside, PARAMS.hook_inside);
    }

    public void setHookInside() {
        PARAMS.hook_target = PARAMS.hook_inside;
    }

    public void setHookOutside() {
        PARAMS.hook_target = PARAMS.hook_outside;
    }

    public void setHookIdle() {
        PARAMS.hook_target = PARAMS.hook_idle;
    }

    public void setHookToAttach() {
        PARAMS.hook_target = 0.4;
    }

    /////////////////////////////////////////////////////////////

    public void setWenchPower(double power) {
        PARAMS.wench_power = power;
    }

    /////////////////////////////////////////////////////////////

    public void run() {
        // Drive motors
        double l = pid_left.calculate(elevator_left.getCurrentPosition(), PARAMS.target_height) + PARAMS.kG;
        double r = pid_right.calculate(elevator_right.getCurrentPosition(), PARAMS.target_height) + PARAMS.kG;

        l = Math.min(Math.max(PARAMS.minSpeed, l), PARAMS.maxSpeed);
        r = Math.min(Math.max(PARAMS.minSpeed, r), PARAMS.maxSpeed);

        if (PARAMS.target_height == 0.0 && elevator_left.getCurrentPosition() <= 100 && elevator_right.getCurrentPosition() <= 100 && PARAMS.elevator_enable_auto_reset) {
            l = 0;
            r = 0;

            if (!reset_encoder_timer.isTimerOn()) {
                reset_encoder_timer.start();

            }

            if (reset_encoder_timer.done()) {
                elevator_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                elevator_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                reset_encoder_timer.pause();
            }
        } else {
            reset_encoder_timer.pause();
        }

        elevator_left.setPower(l);
        elevator_right.setPower(r);

        servo_hook_right.setPosition(PARAMS.hook_target);
        servo_hook_left.setPosition(PARAMS.hook_target);

        wench.setPower(PARAMS.wench_power);
    }

    public void run(TelemetryPacket packet) {
        run();
        packet.put("elevator_left", elevator_left.getCurrentPosition());
        packet.put("elevator_right", elevator_right.getCurrentPosition());
        packet.put("elevator_timer", reset_encoder_timer.remainingTime());
        packet.put("elevator_wrench", wench.getCurrentPosition());
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.125;

        // Normal PID constants
        public double kP = 0.01;
        public double kI = 0.0;
        public double kD = 0.0001;

        public double maxSpeed = 1.0;
        public double minSpeed = -1.0;//-0.3;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 4000.0;

        public double target_height = 0.0;

        public double hook_outside = 0.025;
        public double hook_inside = 1.0;
        public double hook_idle = 0.375;
        public double hook_target = hook_outside;

        public boolean elevator_enable_auto_reset = true;

        public double elevator_pose_ready_to_hook = 2000.0;

        public double wench_power = 0.0;
    }
}
