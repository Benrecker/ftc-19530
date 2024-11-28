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
@Autonomous(name = "4 box butter")
public class fourboxbutter extends LinearOpMode {
    private Elevator elevator;
    private Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);


        //in front of box traj

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-33.05, -62.70, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))//position that we will go to after every pick up
                .build();


        //once ele is up, forward a little bit

        TrajectorySequence trajdriveforward = drive.trajectorySequenceBuilder(trajectory0.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(5, 7, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .forward(4)
                .build();
        //turn robot to block 1

        TrajectorySequence trajpickupB1 = drive.trajectorySequenceBuilder(trajdriveforward.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-53.26, -51.64, Math.toRadians(82.00)))
                .build();


        TrajectorySequence trajbeforeforwardb1 = drive.trajectorySequenceBuilder(trajpickupB1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))//position that we will go to after every pick up
                .build();
        //turn robot to block 2

        TrajectorySequence trajpickupB2 = drive.trajectorySequenceBuilder(trajdriveforward.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-59.46, -50.90, Math.toRadians(90.00)))
                .build();

        TrajectorySequence trajbeforeforwardb2 = drive.trajectorySequenceBuilder(trajpickupB2.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))//position that we will go to after every pick up
                .build();

        //turn robot to block 3


        TrajectorySequence trajpickupB3 = drive.trajectorySequenceBuilder(trajdriveforward.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-57.69, -50.75, Math.toRadians(118.84)))
                .build();

        TrajectorySequence trajbeforeforwardb3 = drive.trajectorySequenceBuilder(trajpickupB3.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))//position that we will go to after every pick up
                .build();

        TrajectorySequence trajpark = drive.trajectorySequenceBuilder(new Pose2d(-33.05, -62.70, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))//position that we will go to after every pick up
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


            //first block in box
            if (state == 0) {
                if (!drive.isBusy()) {
                    elevator.setHeight(4000);
                }
                if (elevator.atTarget()) {
                    drive.followTrajectorySequenceAsync(trajdriveforward);

                    if (!drive.isBusy()) {
                        grabber.intakeOut();
                        timer.start();
                        if (timer.done()) {
                            elevator.setHeight(0);
                            state++;
                        }
                    }


                }
                //intake block 2
            } else if (state == 1) {
                if (!drive.isBusy() && elevator.atTarget(1000)) {
                    grabber.armToFloor();
                    grabber.intakeIn();
                    grabber.slideToOutside();
                    drive.followTrajectorySequenceAsync(trajpickupB1);
                    timer.start();
                    state++;

                }

                //disgarging block
            } else if (state == 2) {
                if (!drive.isBusy() && timer.done()) {
                    grabber.armToInside();
                    grabber.slideToInside();
                    grabber.intakeStop();
                    drive.followTrajectorySequenceAsync(trajbeforeforwardb1);
                    timer2.start();
                    if (!drive.isBusy() && timer2.done()) {
                        elevator.setHeight(4000);
                    }
                    if (elevator.atTarget()) {
                        drive.followTrajectorySequenceAsync(trajdriveforward);

                        if (!drive.isBusy()) {
                            grabber.intakeOut();
                            timer.start();
                            if (timer.done()) {
                                elevator.setHeight(0);
                                state++;
                            }
                        }
                    }

                } else if (state == 3) {
                    if (!drive.isBusy() && elevator.atTarget(1000)) {
                        grabber.armToFloor();
                        grabber.intakeIn();
                        grabber.slideToOutside();
                        drive.followTrajectorySequenceAsync(trajpickupB2);
                        timer.start();
                        state++;


                    }
                } else if (state == 4) {
                    if (!drive.isBusy() && timer.done()) {
                        grabber.armToInside();
                        grabber.slideToInside();
                        grabber.intakeStop();
                        drive.followTrajectorySequenceAsync(trajbeforeforwardb2);
                        timer2.start();
                        if (!drive.isBusy() && timer2.done()) {
                            elevator.setHeight(4000);
                        }
                        if (elevator.atTarget()) {
                            drive.followTrajectorySequenceAsync(trajdriveforward);

                            if (!drive.isBusy()) {
                                grabber.intakeOut();
                                timer.start();
                                if (timer.done()) {
                                    elevator.setHeight(0);
                                    state++;
                                }
                            }
                        }
                    } else if (state == 5) {
                        if (!drive.isBusy() && elevator.atTarget(1000)) {
                            grabber.armToFloor();
                            grabber.intakeIn();
                            grabber.slideToOutside();
                            drive.followTrajectorySequenceAsync(trajpickupB3);
                            timer.start();
                            state++;
                        }
                    }
                }

            } else if (state == 6) {
                if (!drive.isBusy() && timer.done()) {
                    grabber.armToInside();
                    grabber.slideToInside();
                    grabber.intakeStop();
                    drive.followTrajectorySequenceAsync(trajbeforeforwardb3);
                    timer2.start();
                    if (!drive.isBusy() && timer2.done()) {
                        elevator.setHeight(4000);
                    }
                    if (elevator.atTarget()) {
                        drive.followTrajectorySequenceAsync(trajdriveforward);
                        if (!drive.isBusy()) {
                            grabber.intakeOut();
                            timer.start();
                            if (timer.done()) {
                                elevator.setHeight(0);
                                state++;
                                if(elevator.atTarget()) {
                                    drive.turn((Math.PI)/4);
                                }
                            }
                        }
                    }
                    //else if (state == 7) {
//                if (timer.done()) {
//                    state++;
//                    drive.followTrajectorySequenceAsync(traj5);
//                    elevator.setHeight(4000);
//                }
//            } else if (state == 8) {
//                if (!drive.isBusy()) {
//                    state++;
//                    drive.followTrajectorySequenceAsync(traj1);
//                }
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
}

