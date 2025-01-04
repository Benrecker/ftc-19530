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
@Autonomous(name = "hybrid blue" )
public class DaHybridBlue extends LinearOpMode {
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

        TrajectorySequence trajectorydrivetosubhigh = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -64.00, Math.toRadians(180.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .lineToLinearHeading(new Pose2d(-14.00, -31.0, Math.toRadians(190.00)))
                .build();

        TrajectorySequence trajectorydropspeicmen = drive.trajectorySequenceBuilder(trajectorydrivetosubhigh.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 20, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-22.00, -40.00, Math.toRadians(155.00)))
                .lineToLinearHeading(new Pose2d(-28.00, -37.00, Math.toRadians(155.00)))
                .build();

        TrajectorySequence trajectorydropB1 = drive.trajectorySequenceBuilder(trajectorydropspeicmen.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-55.5, -55.5, Math.toRadians(45.00)))
                .build();

        TrajectorySequence trajectorypickupB2 = drive.trajectorySequenceBuilder(new Pose2d(-56.50, -56.50, Math.toRadians(45.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-59.50, -50.00, Math.toRadians(90.00)))
                .build();

        TrajectorySequence trajectorydropB2 = drive.trajectorySequenceBuilder(new Pose2d(-59.50, -50.00, Math.toRadians(90.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-55.5, -55.5, Math.toRadians(45.00)))
                .build();


        TrajectorySequence trajectorypickupB3 = drive.trajectorySequenceBuilder(new Pose2d(-56.50, -56.50, Math.toRadians(45.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-59.50, -50.00, Math.toRadians(112.00)))
                .build();

        TrajectorySequence trajectorydropB3 = drive.trajectorySequenceBuilder(new Pose2d(-59.50, -50.00, Math.toRadians(115.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-55.5, -55.5, Math.toRadians(45.00)))
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(new Pose2d(-54.00, -55.00, Math.toRadians(45.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-31.00, -13.00, Math.toRadians(35.00)))
                .build();


        drive.setPoseEstimate(trajectorydrivetosubhigh.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(trajectorydrivetosubhigh);

        int state = 0;

        Timing.Timer timer = new Timing.Timer(1);
        Timing.Timer timer2 = new Timing.Timer(1 / 2);
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


            //first speicmen to the sub
            if (state == 0) {
                elevator.setHeight(1650);
                if (!drive.isBusy()) {
                    state++;
                }
            } else if (state == 1) {
                if (elevator.atTarget() && !drive.isBusy()) {
                    elevator.setHeight(1150);
                    state++;
                    timer.start();
                }
                //drops the speicmen and drives to B1
            } else if (state == 2) {
                if (elevator.atTarget() && timer.done()) {
                    drive.followTrajectorySequenceAsync(trajectorydropspeicmen);
                    elevator.setHeight(0);
                    grabber.armToFloor();
                    grabber.intakeIn();
                    state++;
                }
            } else if (state == 3) {
                //intake B1
                if (!drive.isBusy()) {
                    grabber.slideToPercent(0.9);
                    state++;
                    timer.start();
                }

            } else if (state == 4) {
                if (timer.done()) {
                    grabber.intakeInSlow();
                    grabber.armToInside();
                    grabber.slideToInside();
                    drive.followTrajectorySequenceAsync(trajectorydropB1);
                    state++;
                }
                //drop the B1 block
            } else if (state == 5) {
                if (!drive.isBusy()) {
                    elevator.setHeight(4150);
                    state++;
                }
            } else if (state == 6) {
                if (elevator.atTarget()) {
                    grabber.intakeOut();
                    state++;
                    timer.start();
                }
            } else if (state == 7) {
                if (timer.done()) {
                    elevator.setHeight(0);
                    state++;
                }
            } else if (state == 8) {
                //drive to intake B2
                if (elevator.atTarget(1500)) {
                    grabber.armToFloor();
                    grabber.intakeIn();
                    drive.followTrajectorySequenceAsync(trajectorypickupB2);
                    state++;

                }
            } else if (state == 9) {
                //intake B2
                if (!drive.isBusy()) {
                    grabber.slideToPercent(0.9);
                    timer.start();
                    state++;
                }
            } else if (state == 10) {
                if (timer.done()) {
                    grabber.intakeInSlow();
                    drive.followTrajectorySequenceAsync(trajectorydropB2);
                    grabber.armToInside();
                    grabber.slideToInside();
                    state++;
                }
            } else if (state == 11) {
                //drop B2
                if (!drive.isBusy()) {
                    elevator.setHeight(4150);
                    state++;
                }
            } else if (state == 12) {
                if (elevator.atTarget()) {
                    grabber.intakeOut();
                    timer.start();
                    state++;
                }
            } else if (state == 13) {
                if (timer.done()) {
                    grabber.intakeStop();
                    elevator.setHeight(0);
                    state++;
                }
            } else if (state == 14) {
                //drive to B3
                if (elevator.atTarget(1500)) {
                    grabber.armToFloor();
                    grabber.intakeIn();
                    drive.followTrajectorySequenceAsync(trajectorypickupB3);
                    state++;
                }
            } else if (state == 15) {
                //intake B3
                if (!drive.isBusy()) {
                    grabber.slideToPercent(1);
                    timer.start();
                    state++;
                }
            } else if (state == 16) {
                if (timer.done()) {
                    grabber.intakeInSlow();
                    grabber.armToInside();
                    grabber.slideToInside();
                    drive.followTrajectorySequenceAsync(trajectorydropB3);
                    state++;
                }
            } else if (state == 17) {
                //drop B3
                if (!drive.isBusy()) {
                    elevator.setHeight(4150);
                    state++;
                }
            } else if (state == 18) {
                if (elevator.atTarget()) {
                    grabber.intakeOut();
                    timer.start();
                    state++;
                }
            } else if (state == 19) {
                if (timer.done()) {
                    grabber.intakeStop();
                    elevator.setHeight(0);
                    state++;
                }
            } else if (state == 20) {
                //PARK
                if (elevator.atTarget(1000)) {
                    drive.followTrajectorySequenceAsync(Park);
                    timer.start();
                    state++;
                }
            }
        }
    }
}




