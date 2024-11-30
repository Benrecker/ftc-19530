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
@Autonomous(name = "4 box butter red")
public class fourboxbutterRed extends LinearOpMode {
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

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-33.05, -62.70, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))//position that we will go to after every pick up
                .build();


        //once ele is up, forward a little bit

        TrajectorySequence trajdriveforward = drive.trajectorySequenceBuilder(trajectory0.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(5, 7, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .back(2.5) //distance when dropping
                .build();

        //turn robot to block 1

        TrajectorySequence trajpickupB1 = drive.trajectorySequenceBuilder(trajdriveforward.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-53.26, -51.64, Math.toRadians(92))) //may need to change this angle
                .build();


        TrajectorySequence trajbeforeforwardb1 = drive.trajectorySequenceBuilder(trajpickupB1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))
                .build();
        //turn robot to block 2

        TrajectorySequence trajpickupB2 = drive.trajectorySequenceBuilder(trajdriveforward.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-59.46, -50.90, Math.toRadians(100))) //may need to change this angle
                .build();

        TrajectorySequence trajbeforeforwardb2 = drive.trajectorySequenceBuilder(trajpickupB2.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(45.00)))
                .build();

        //turn robot to block 3


        TrajectorySequence trajpickupB3 = drive.trajectorySequenceBuilder(trajdriveforward.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-57.69, -50.75, Math.toRadians(123.0))) //may need to change this angle
                .build();

        TrajectorySequence trajbeforeforwardb3 = drive.trajectorySequenceBuilder(trajpickupB3.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-54.74, -55.18, Math.toRadians(78)))
                .build();


        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(trajectory0);

        int state = 0;

        Timing.Timer timer = new Timing.Timer(1);
        Timing.Timer timer2 = new Timing.Timer(1 / 2);
        Timing.Timer timer3 = new Timing.Timer(3/2);



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


            //first block in box
            if (state == 0) {
                if (!drive.isBusy()) {
                    elevator.setHeight(4000);
                    state++;
                }
            } else if (state == 1) {
                if (elevator.atTarget()) {
                    drive.followTrajectorySequenceAsync(trajdriveforward);
                    state++;
                }
            } else if (state == 2) {
                if (!drive.isBusy()) {
                    grabber.intakeOut();
                    timer.start();
                    state++;
                }
            } else if (state == 3) {
                if (timer.done()) {
                    elevator.setHeight(0);
                    grabber.intakeStop();
                    state++;
                    timer.start();
                }
                //intake block 2
            } else if (state == 4) {
                if (timer.done() && elevator.atTarget(1000)) {
                    drive.followTrajectorySequenceAsync(trajpickupB1);
                    timer.start();
                    state++;

                }
                //disgarging block

            } else if (state == 5) {
                if (!drive.isBusy() && timer.done()) {
                    grabber.armToFloor();
                    grabber.intakeIn();
                    grabber.slideToPercent(.6);
                    timer.start();
                    state++;
                }
            } else if (state == 6) {
                if(timer.done()) {
                    drive.followTrajectorySequenceAsync(trajbeforeforwardb1);
                    grabber.armToInside();
                    grabber.slideToInside();
                    grabber.intakeStop();
                    timer.start();
                    state++;
                }
            }
            else if (state == 7) {
                if (!drive.isBusy() && timer.done()) {
                    elevator.setHeight(4000);
                    state++;
                }
            } else if (state == 8) {
                if (elevator.atTarget()) {
                    drive.followTrajectorySequenceAsync(trajdriveforward);
                    state++;
                    timer.start();
                }
            } else if (state == 9) {
                if (!drive.isBusy() && timer.done()) {
                    grabber.intakeOut();
                    timer.start();
                    state++;
                }
            } else if (state == 10) {
                if (timer.done()) {
                    elevator.setHeight(0);
                    grabber.intakeStop();
                    state++;
                    timer.start();
                }
            } else if (state == 11) {
                if (!drive.isBusy() && elevator.atTarget(2500)) {
                    drive.followTrajectorySequenceAsync(trajpickupB2);
                    timer.start();
                    state++;
                }
            } else if (state == 12) {
                if (!drive.isBusy() && timer.done()) {
                    grabber.armToFloor();
                    grabber.intakeIn();
                    grabber.slideToPercent(0.5);
                    timer.start();
                    state++;
                }
            }else if (state == 13) {
                if (timer.done()) {
                    drive.followTrajectorySequenceAsync(trajbeforeforwardb2);
                    grabber.armToInside();
                    grabber.slideToInside();
                    grabber.intakeIn();
                    timer.start();
                    state++;
                }
            }else if (state == 14) {
                if (!drive.isBusy() && timer.done()){
                    elevator.setHeight(4000);
                    state++;
                }
            }else if (state == 15) {
                if (elevator.atTarget()) {
                    drive.followTrajectorySequenceAsync(trajdriveforward);
                    grabber.intakeStop();
                    state++;
                }
            }else if (state == 16) {
                if  (!drive.isBusy()) {
                    grabber.intakeOut();
                    state++;
                    timer.start();
                }
            }else if (state ==17) {
                if (timer.done()) {
                    elevator.setHeight(0);
                    grabber.intakeStop();
                    state++;
                }
            } else if (state == 18) {
                if (!drive.isBusy() && elevator.atTarget(2500)) {
                    drive.followTrajectorySequenceAsync(trajpickupB3);
                    timer.start();
                    state++;
                }
            } else if (state == 19) {
                if (!drive.isBusy() && timer.done()) {
                    grabber.armToFloor();
                    grabber.intakeIn();
                    grabber.slideToPercent(0.75);
                    timer.start();
                    state++;
                }
            }else if (state == 20) {
                if (timer.done() && (!drive.isBusy())) {
                    drive.followTrajectorySequenceAsync(trajbeforeforwardb3);
                    grabber.armToInside();
                    grabber.slideToInside();
                    grabber.intakeIn();
                    timer.start();
                    state++;
                }
            }else if (state == 21) {
                if (!drive.isBusy() && timer.done()){
                    grabber.intakeOut();
                    state++;
                }
//            }else if (state == 22) {
//                if (elevator.atTarget()) {
//                    drive.followTrajectorySequenceAsync(trajdriveforward);
//                    state++;
//                }
//            }else if (state == 23) {
//                if  (!drive.isBusy()) {
//                    grabber.intakeOut();
//                    state++;
//                    timer.start();
//                }
//            }else if (state == 24) {
//                if (timer.done()) {
//                    elevator.setHeight(0);
//                    grabber.intakeStop();
//                    state++;
//                    timer2.start();
//                }
//            }else if (state == 25) {
//                if (timer2.done()) {
//                    drive.followTrajectorySequenceAsync(trajpark);
//                }
            }
        }
    }
}



