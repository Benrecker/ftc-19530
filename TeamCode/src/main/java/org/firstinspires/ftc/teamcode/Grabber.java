package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Grabber {
    public static Params PARAMS = new Params();

    ServoImplEx servo_arm_right;
    ServoImplEx servo_arm_left;

    ServoImplEx servo_slide_right;
    ServoImplEx servo_slide_left;

    CRServo servo_intake_left;
    CRServo servo_intake_right;

    public Grabber(HardwareMap hardwareMap) {
        servo_arm_left = hardwareMap.get(ServoImplEx.class, "servoWristLeft");
        servo_arm_right = hardwareMap.get(ServoImplEx.class, "servoWristRight");

        servo_intake_right = hardwareMap.get(CRServo.class, "servoIntakeLeft");
        servo_intake_left = hardwareMap.get(CRServo.class, "servoIntakeRight");

        servo_slide_right = hardwareMap.get(ServoImplEx.class, "servoSlideRight");
        servo_slide_left = hardwareMap.get(ServoImplEx.class, "servoSlideLeft");

        ///////////////////////////////////

        servo_arm_left.setDirection(Servo.Direction.REVERSE);

        servo_intake_right.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_slide_left.setDirection(Servo.Direction.REVERSE);

        ///////////////////////////////////

        servo_arm_left.setPwmEnable();
        servo_arm_right.setPwmEnable();

        servo_slide_left.setPwmEnable();
        servo_slide_right.setPwmEnable();

        ///////////////////////////////////

        servo_slide_left.setPosition(PARAMS.slide_goal);
        servo_slide_right.setPosition(PARAMS.slide_goal);

        servo_arm_left.setPosition(PARAMS.arm_goal);
        servo_arm_right.setPosition(PARAMS.arm_goal);

        servo_intake_right.setPower(PARAMS.intake_power);
        servo_intake_left.setPower(PARAMS.intake_power);
    }

    ////////////////////////////////////////////////////////////////////////

    public void intakeOut() {
        PARAMS.intake_power = 1.0;
    }

    public void intakeStop() {
        PARAMS.intake_power = 0.0;
    }

    public void intakeIn() {
        PARAMS.intake_power = -1.0;
    }

    public void intakeSetPower(double power) {
        PARAMS.intake_power = clamp(power, -1.0, 1.0);
    }

    ////////////////////////////////////////////////////////////////////////

    public void armToFloor() {
        PARAMS.arm_goal = PARAMS.pose_arm_floor;
    }

    public void armToInside() {
        PARAMS.arm_goal = PARAMS.pose_arm_inside;
    }

    public void armToPrepareToHook( ) {PARAMS.arm_goal = PARAMS.pose_arm_prepare_hook; }

    public void armToHook() {
        PARAMS.arm_goal = PARAMS.pose_arm_hook;
    }

    /**
     * Control the arm by providing a percent where 0% is inside and 100% is on the floor
     * @param percent The percent to travel to
     */
    public void armToPercent(double percent) {
        PARAMS.arm_goal = clamp(map(percent, 0.0, 1.0, PARAMS.pose_arm_floor, PARAMS.pose_arm_inside), PARAMS.pose_arm_floor, PARAMS.pose_arm_inside);
    }

    ////////////////////////////////////////////////////////////////////////

    public void slideToInside() {
        PARAMS.slide_goal = PARAMS.pose_slide_in;
    }

    public void slideToOutside() {
        PARAMS.slide_goal = PARAMS.pose_slide_out;
    }

    /**
     * Control the slide by providing a percent where 0% is inside and 100% is out
     * @param percent The percent to travel to
     */
    public void slideToPercent(double percent) {
        PARAMS.slide_goal = clamp(map(percent, 0.0, 1.0, PARAMS.pose_slide_in, PARAMS.pose_slide_out), PARAMS.pose_slide_in, PARAMS.pose_slide_out);
    }

    ////////////////////////////////////////////////////////////////////////

    public void run() {
        servo_slide_left.setPosition(PARAMS.slide_goal);
        servo_slide_right.setPosition(PARAMS.slide_goal);

        servo_arm_left.setPosition(PARAMS.arm_goal);
        servo_arm_right.setPosition(PARAMS.arm_goal);

        servo_intake_right.setPower(PARAMS.intake_power);
        servo_intake_left.setPower(PARAMS.intake_power);
    }

    public void run(TelemetryPacket packet) {
        run();

        // For debugging
        packet.put("grabber_slide_left", servo_slide_left.getPosition());
        packet.put("grabber_slide_right", servo_slide_right.getPosition());
        packet.put("grabber_arm_left", servo_arm_left.getPosition());
        packet.put("grabber_arm_right", servo_arm_right.getPosition());
        packet.put("grabber_intake_left", servo_intake_left.getPower());
        packet.put("grabber_intake_right", servo_intake_right.getPower());
    }

    ////////////////////////////////////////////////////////////////////////

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static class Params {
        public double pose_arm_inside = 0.95;

        public double pose_arm_prepare_hook = 0.45;
        public double pose_arm_hook = 0.68  ;
        public double pose_arm_floor = 0.33;

        public double pose_slide_out = 0.185; //old value was 0.175
        public double pose_slide_in = 0.0;

        public double arm_goal = pose_arm_inside;
        public double slide_goal = pose_slide_in;
        public double intake_power = 0.0;
    }
}
