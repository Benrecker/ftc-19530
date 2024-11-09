package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Elevator Test")
@Config
public class ElevatorTest extends OpMode {

    public static ElevatorTest.Params PARAMS = new ElevatorTest.Params();
    private FtcDashboard dash = FtcDashboard.getInstance();

    private DcMotorEx elevatorLeft;
    private DcMotorEx elevatorRight;
    private Elevator elevator;

    PIDController pidLeft;
    PIDController pidRight;

    private double targetHeight = 0.0;

    static double l_smoothed = 0.0;
    static double r_smoothed = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        // Set idle behavior
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motor direction
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidLeft = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        pidRight = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);

    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        targetHeight += gamepad1.right_trigger * PARAMS.controlConst;
        targetHeight -= gamepad1.left_trigger * PARAMS.controlConst;

//            elevatorLeft.setPower(gamepad.right_trigger + PARAMS.kG);
//            elevatorRight.setPower(gamepad.right_trigger + PARAMS.kG);

        targetHeight = Math.min(Math.max(PARAMS.encoderMinimum, targetHeight), PARAMS.encoderMaximum);

        pidLeft.setP(PARAMS.kP);
        pidLeft.setI(PARAMS.kI);
        pidLeft.setD(PARAMS.kD);

        pidRight.setP(PARAMS.kP);
        pidRight.setI(PARAMS.kI);
        pidRight.setD(PARAMS.kD);

        // Drive motors
        double l = pidLeft.calculate(elevatorLeft.getCurrentPosition(), targetHeight) + PARAMS.kG;
        double r = pidRight.calculate(elevatorRight.getCurrentPosition(), targetHeight) + PARAMS.kG;

        l_smoothed = PARAMS.alpha * l_smoothed + (1-PARAMS.alpha) * l;
        r_smoothed = PARAMS.alpha * r_smoothed + (1-PARAMS.alpha) * r;

        l = Math.min(Math.max(PARAMS.minSpeed, l_smoothed), PARAMS.maxSpeed);
        r = Math.min(Math.max(PARAMS.minSpeed, r_smoothed), PARAMS.maxSpeed);


        packet.put("pid_left", l_smoothed);
        packet.put("pid_right", r_smoothed);
        packet.put("encoder_left", elevatorLeft.getCurrentPosition());
        packet.put("encoder_right", elevatorRight.getCurrentPosition());
        packet.put("elevator-target", targetHeight);
        elevatorLeft.setPower(l);
        elevatorRight.setPower(r);

        dash.sendTelemetryPacket(packet);
    }

    public static class Params {
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward
        public double kG = 0.175;

        // Normal PID constants
        public double kP = 0.005;
        public double kI = 0.01;
        public double kD = 0.0005;

        public double maxSpeed = 1.0;
        public double minSpeed = -0.01;

        public double alpha = 0.1;

        public double controlConst = 20.0;

        // Elevator height constants (in units of encoder ticks)
        public double encoderMinimum = 0.0;
        public double encoderMaximum = 4000.0;
    }

}
