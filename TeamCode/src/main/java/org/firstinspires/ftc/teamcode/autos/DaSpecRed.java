package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous(name = "new ROC HAWK KILLA" )
public class DaSpecRed extends LinearOpMode {
    private Elevator elevator;
    private Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);


        //WHEN GETTING TO COMP, COME HERE
        //run auto and if things need to be changed just go the traj that needs to be changed.
        //If its not driving far enough on the drop, go to "trajdriveforawrd" and change the distance.
        //all of the angles may need to be changed, when robot is facing the oppsite the bucket, that is 0 degrees
        //so if the angle is under shooting, add degrees, oppsite for over shooting.

        //corner to left is (-62,-62). if you need to change x, y vaules


        //in front of box traj

        TrajectorySequence drivesubhigh = drive.trajectorySequenceBuilder(new Pose2d(9.00, -63.00, Math.toRadians(0.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(14.00, -32, Math.toRadians(350.00)))
                .build();


        TrajectorySequence pickupB1 = drive.trajectorySequenceBuilder(drivesubhigh.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(20.0, -44.00, Math.toRadians(47.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(34.50, -44.00, Math.toRadians(47.00)))
                .build();

//        TrajectorySequence dropoffB1 = drive.trajectorySequenceBuilder(pickupB1.end())
//                .lineToLinearHeading(new Pose2d(31.50, -44.00, Math.toRadians(315.00)))
//                .build();
//
//        TrajectorySequence pickupB2 = drive.trajectorySequenceBuilder(pickupB1.end())
//                .lineToLinearHeading(new Pose2d(42.50, -43.00, Math.toRadians(47.00)))
//                .build();
//
//        TrajectorySequence dropoffB2 = drive.trajectorySequenceBuilder(pickupB2.end())
//                .lineToLinearHeading(new Pose2d(41.50, -44.00, Math.toRadians(315.00)))
//                .build();
//
//        TrajectorySequence pickupB3 = drive.trajectorySequenceBuilder(pickupB2.end())
//                .lineToLinearHeading(new Pose2d(52.50, -43.00, Math.toRadians(35.00)))
//                .build();


//        TrajectorySequence dropoffB3 = drive.trajectorySequenceBuilder(pickupB3.end())
//                .lineToLinearHeading(new Pose2d(51.50, -44.00, Math.toRadians(35.00)))
//                .build();

        TrajectorySequence pickupspec1 = drive.trajectorySequenceBuilder(pickupB1.end())
                .lineToLinearHeading(new Pose2d(35.00, -58.00, Math.toRadians(270.00)))
//              .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

        TrajectorySequence driveforward1 = drive.trajectorySequenceBuilder(pickupspec1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(10, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

        TrajectorySequence dropspec1 = drive.trajectorySequenceBuilder(driveforward1.end())
                .lineToLinearHeading(new Pose2d(35.00, -59.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-4.00, -36.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-4.00, -29.50, Math.toRadians(90.00)))
                .build();

        TrajectorySequence pickupspec2 = drive.trajectorySequenceBuilder(dropspec1.end())
                .lineToLinearHeading(new Pose2d(-4, -36.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(35.00, -58.00, Math.toRadians(270.00)))
//              .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

        TrajectorySequence driveforward2 = drive.trajectorySequenceBuilder(pickupspec2.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(10, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

        TrajectorySequence dropspec2 = drive.trajectorySequenceBuilder(driveforward2.end())
                .lineToLinearHeading(new Pose2d(35.00, -59.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-1.00, -36.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-1.00, -29.50, Math.toRadians(90.00)))
                .build();

        TrajectorySequence pickupspec3 = drive.trajectorySequenceBuilder(dropspec2.end())
                .lineToLinearHeading(new Pose2d(-1, -36.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(35.00, -58.00, Math.toRadians(270.00)))
//              .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

        TrajectorySequence driveforward3 = drive.trajectorySequenceBuilder(pickupspec3.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(10, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

        TrajectorySequence dropspec3 = drive.trajectorySequenceBuilder(driveforward3.end())
                .lineToLinearHeading(new Pose2d(35.00, -59.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(2.00, -36.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(2.00, -29.50, Math.toRadians(90.00)))
                .build();

//        TrajectorySequence pickupspec4 = drive.trajectorySequenceBuilder(dropspec3.end())
//                .lineToLinearHeading(new Pose2d(2, -36.50, Math.toRadians(90.00)))
//                .lineToLinearHeading(new Pose2d(35.00, -58.00, Math.toRadians(270.00)))
////              .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
//                .build();
//
//        TrajectorySequence dropspec4 = drive.trajectorySequenceBuilder(pickupspec4.end())
//                .lineToLinearHeading(new Pose2d(35.00, -59.00, Math.toRadians(270.00)))
//                .lineToLinearHeading(new Pose2d(5.00, -36.50, Math.toRadians(90.00)))
//                .lineToLinearHeading(new Pose2d(5.00, -31.50, Math.toRadians(90.00)))
//                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(dropspec3.end())
                .lineToLinearHeading(new Pose2d(35, -64.00, Math.toRadians(90.00)))
//                .lineToLinearHeading(new Pose2d(35.00, -59.00, Math.toRadians(270.00)))
//                .lineToLinearHeading(new Pose2d(35.00, -64.00, Math.toRadians(270.00)))
                .build();

//extend slide to touch bar


        drive.setPoseEstimate(drivesubhigh.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(drivesubhigh);

        int state = 0;

        Timing.Timer timer = new Timing.Timer(1);
        Timing.Timer timer2 = new Timing.Timer(1);
        Timing.Timer timer3 = new Timing.Timer(3 / 2);


        while (opModeIsActive() && !isStopRequested()) {


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("state", state);
            telemetry.update();
            elevator.run();
            grabber.run();
            drive.update();


//            first speicmen to the sub
            if (state == 0) {
                elevator.setHeight(1850);
                if (!drive.isBusy()) {
                    state++;
                }
            } else if (state == 1) {
                if (elevator.atTarget()) {
                    elevator.setHeight(1451);
                    state++;
                }
            } else if (state == 2) {
                if (elevator.atTarget(200)) {
                    elevator.setHeight(0);
                    drive.followTrajectorySequenceAsync(pickupB1);
                    grabber.slideToPercent(.9);
                    grabber.armToFloor();
                    grabber.intakeIn();
                    state++;
                }
            } else if (state == 3) {
                if (!drive.isBusy()) {
                    state++;
                    timer.start();
                }
            } else if (state == 4) {
                if (timer.done()) {
                    grabber.intakeInSlow();
                    drive.turn(Math.toRadians(-95));
                    state++;
                }
                //disgarging block

            } else if (state == 5) {
                if (!drive.isBusy()) {
                    grabber.intakeOut();
                    timer.start();
                    state++;
                }
            } else if (state == 6) {
                if (timer.done()) {
                    grabber.slideToInside();
                    grabber.armToInside();
                    grabber.intakeStop();
                    timer.start();
                    state++;
                }
            } else if (state == 7) {
                if (timer.done()) {
                    drive.followTrajectorySequenceAsync(pickupspec1);
                    state++;
                }
            } else if (state == 8) {
                if (!drive.isBusy()) {
                    drive.followTrajectorySequenceAsync(driveforward1);
                    state++;
                }
            } else if (state == 9) {
                if (!drive.isBusy()) {
                    grabber.clamp_off();
                    timer.start();
                    state++;
                }
//            } else if (state == 10) {
//                if (elevator.atTarget()) {
//                    drive.followTrajectorySequenceAsync(pickupspec2);
//                    elevator.setHeight(0);
//                    state++;
//                }
//            } else if (state == 11) {
//                if (!drive.isBusy()) {
//                    drive.followTrajectorySequenceAsync(driveforward2);
//                    if (!drive.isBusy()) {
//                        grabber.clamp_off();
//                        state++;
//                        timer.start();
//                    }
//                }
//            } else if (state == 12) {
//                if (timer.done()) {
//                    drive.followTrajectorySequenceAsync(dropspec2);
//                    elevator.setHeight(1850);
//                    state++;
//                }
//            } else if (state == 13) {
//                if (!drive.isBusy()) {
//                    elevator.setHeight(1450);
//                    state++;
//                }
//            } else if (state == 14) {
//                if (elevator.atTarget()) {
//                    drive.followTrajectorySequenceAsync(pickupspec2);
//                    elevator.setHeight(0);
//                    state++;
//                }
//            } else if (state == 15) {
//                if (!drive.isBusy()) {
//                    drive.followTrajectorySequenceAsync(driveforward3);
//                    if (!drive.isBusy()) {
//                        grabber.clamp_off();
//                        state++;
//                        timer.start();
//                    }
//                }
//            } else if (state == 16) {
//                if (timer.done()) {
//                    drive.followTrajectorySequenceAsync(dropspec3);
//                    elevator.setHeight(1850);
//                    state++;
//                }
//            } else if (state == 17) {
//                if (!drive.isBusy()) {
//                    elevator.setHeight(1450);
//                    state++;
//                }
                //else if (state == 18) {
//                if (elevator.atTarget()) {
//                    drive.followTrajectorySequenceAsync(pickupspec4);
//                    drive.followTrajectorySequenceAsync(driveforward);
//                    elevator.setHeight(0);
//                    grabber.clamp_on();
//                    state++;
//                }
//            } else if (state == 19) {
//                if (!drive.isBusy()) {
//                    grabber.clamp_off();
//                    state++;
//                    timer.start();
//                }
//            } else if (state == 20) {
//                if (timer.done()) {
//                    drive.followTrajectorySequenceAsync(dropspec4);
//                    elevator.setHeight(1850);
//                    state++;
//                }
//            } else if (state == 21) {
//                if (!drive.isBusy()) {
//                    elevator.setHeight(1450);
//                    state++;
//            } else if (state == 18) {
//                if (elevator.atTarget()) {
//                    drive.followTrajectorySequenceAsync(park);
//                    elevator.setHeight(0);
//                    grabber.clamp_on();
//                    state++;
//                }
//            }
            }
        }
    }
}







