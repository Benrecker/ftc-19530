package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Elevator;
import org.firstinspires.ftc.teamcode.Grabber;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "2 sample box")
public class twosamplebox extends LinearOpMode {
    private Elevator elevator;
    private Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-39.56, -62.50, Math.toRadians(90.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToSplineHeading(new Pose2d(-47.83, -47.83, Math.toRadians(45.00)))
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(trajectory0.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToConstantHeading(new Vector2d(-58.0, -59.6))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToConstantHeading(new Vector2d(-47.83, -47.83))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToSplineHeading(new Pose2d(-47.83, -52.0, Math.toRadians(90)))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(3, 5, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .forward(4)
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToSplineHeading(new Pose2d(-47.83, -47.83, Math.toRadians(45.00)))
                .build();



        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequenceAsync(trajectory0);
        int state = 0;

        Timing.Timer timer = new Timing.Timer(1);

        while (opModeIsActive() && !isStopRequested()) {

            elevator.run();
            grabber.run();
            drive.update();

            if (state == 0) {
                if (!drive.isBusy()) {
                    state++;
                    elevator.setHeight(4000);
                }
            } else if (state == 1) {
                if (elevator.atTarget()) {
                    state++;
                    drive.followTrajectorySequenceAsync(traj1);
                }
            } else if (state == 2) {

                if (!drive.isBusy()) {
                    grabber.intakeOut();
                    drive.followTrajectorySequenceAsync(traj2);
                    state++;
                }
            } else if (state == 3) {
                if (!drive.isBusy()) {
                    grabber.intakeStop();
                    state++;
                    elevator.setHeight(0);
                    drive.followTrajectorySequenceAsync(traj3);
                }
            } else if (state == 4) {
                if (!drive.isBusy() && elevator.atTarget()) {
                    state++;
                    grabber.armToFloor();
                    grabber.intakeIn();
                    timer.start();
                }
            } else if (state == 5) {
                if (timer.done()) {
                    grabber.slideToPercent(0.5);
                    drive.followTrajectorySequenceAsync(traj4);
                    state++;
                    timer.start();
                }
            } else if (state == 6) {
                if (!drive.isBusy() && timer.done()) {
                    state++;
                    grabber.armToInside();
                    grabber.intakeStop();
                    grabber.slideToInside();
                    timer.start();
                }
            } else if (state == 7) {
                if (timer.done()) {
                    state++;
                    drive.followTrajectorySequenceAsync(traj5);
                    elevator.setHeight(4000);
                }
            } else if (state == 8) {
                if (!drive.isBusy()) {
                    state++;
                    drive.followTrajectorySequenceAsync(traj1);
                }
            } else if (state == 9) {
                if (!drive.isBusy()) {
                    grabber.intakeOut();
                    timer.start();
                    state++;
                }
            } else if (state == 10) {
                if (timer.done()) {
                    state++;
                    drive.followTrajectorySequenceAsync(traj2);
                }
            } else if (state == 11) {
                if (!drive.isBusy()) {
                    grabber.intakeStop();
                    elevator.setHeight(0);
                    state++;
                }
            } else if (state == 12) {
                if (elevator.atTarget(500)) {
                    state++;
                    drive.followTrajectorySequenceAsync(traj1);
                }
            }


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("state", state);
            telemetry.update();

        }
    }
}
