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
@Autonomous(name = "2 part specimen and push red")
public class twopartspecimenpushRed extends LinearOpMode {
    private Elevator elevator;
    private Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);


        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(9.32, -62.90, Math.toRadians(90.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineTo(new Vector2d(3.01, -32.25))
                .build();


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(trajectory0.end())
                .lineToLinearHeading(new Pose2d(1.62, -49.72, Math.toRadians(270.00)))
                .lineTo(new Vector2d(37.03, -54.30))
                .build();


        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(10, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineTo(new Vector2d(37.03, -69.85))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(7.52, -53.11))
                .lineToLinearHeading(new Pose2d(10.67, -35.41, Math.toRadians(90.00)))
                .build();


        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineTo(new Vector2d(10.67, -32.25))
                .build();


        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(new Vector2d(0.00, -43.00))
                .lineToSplineHeading(new Pose2d(41.00, -36.00, Math.toRadians(270.00)))
                .lineTo(new Vector2d(45.00, -8.00))
                .lineTo(new Vector2d(55.00, -62.00))
                .lineTo(new Vector2d(40.00, -8.00))
                .build();



        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(trajectory0);

        int state = 0;

        Timing.Timer timer = new Timing.Timer(1);
        Timing.Timer timer2 = new Timing.Timer(1 / 2);


        while (opModeIsActive() && !isStopRequested()) {

            elevator.run();
            grabber.run();
            drive.update();

            if (state == 0) {
                elevator.setHeight(2000);
                if (elevator.atTarget() && !drive.isBusy()) {
                    elevator.setHeight(1500);
                    state++;


                }

            } else if (state == 1) {
                if (elevator.atTarget()) {
                    drive.followTrajectorySequenceAsync(traj1);
                    elevator.setHeight(0);
                    state++;
                    timer.start();
                }
            } else if (state == 2) {
                if (!drive.isBusy() && timer.done()) {
                    drive.followTrajectorySequenceAsync(traj2);
                    state++;
                }
            } else if (state == 3) {
                if (!drive.isBusy()) {
                    elevator.setHeight(200);
                    state++;
                    timer.start();
                }
            } else if (state == 4) {
                if (timer.done()) {
                    drive.followTrajectorySequenceAsync(traj3);
                    state++;
                    timer.start();
                }
            } else if (state == 5) {
                if (!drive.isBusy() && timer.done()) {
                    elevator.setHeight(2000);
                    state++;
                    timer.start();
                }
            }else if (state == 6) {
                if (elevator.atTarget() && timer.done()) {
                    drive.followTrajectorySequenceAsync(traj4);
                    state++;
                    timer.start();
                }
           } else if (state == 7) {
              if (!drive.isBusy() && timer.done()) {
                  elevator.setHeight(1500);
                  timer.start();
                  state++;
               }
            } else if (state == 8) {
                if (timer.done()) {
                    drive.followTrajectorySequenceAsync(traj5);
                    state++;
                    timer.start();
                }
            }else if (state == 9) {
                if (timer.done()) {
                    elevator.setHeight(0);
                }
            }

//            } else if (state == 9) {
//                if (!drive.isBusy()) {
//                    grabber.intakeOut();
//                    timer.start();
//                    state++;
//                }
//            } else if (state == 10) {
//                if (timer.done()) {
//                    state++;
//                    drive.followTrajectorySequenceAsync(traj2);
//                }
//            } else if (state == 11) {
//                if (!drive.isBusy()) {
//                    grabber.intakeStop();
//                    elevator.setHeight(0);
//                    state++;
//                }
//            } else if (state == 12) {
//                if (elevator.atTarget(500)) {
//                    state++;
//                    drive.followTrajectorySequenceAsync(traj1);
//                }


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("state", state);
            telemetry.update();

        }
    }
}



