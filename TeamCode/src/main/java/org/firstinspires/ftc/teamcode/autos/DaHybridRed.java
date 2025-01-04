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
@Autonomous(name = "hybrid red" )
public class DaHybridRed extends LinearOpMode {
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

        TrajectorySequence drivesubhigh = drive.trajectorySequenceBuilder(new Pose2d(-15.00, -64.00, Math.toRadians(180.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(17))
                .lineToLinearHeading(new Pose2d(-14.00, -33.00, Math.toRadians(190.00)))
                .build(); //raise 1900

//drop 150

        TrajectorySequence pickupB1 = drive.trajectorySequenceBuilder(drivesubhigh.end())
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 20, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
//                .lineToLinearHeading(new Pose2d(-22.00, -40.00, Math.toRadians(155.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-38.00, -34.00, Math.toRadians(152.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-55.80, -55.80, Math.toRadians(45.00)))
                .build(); //delay .5 sec then drop 0

//grab block 1

//        TrajectorySequence dropoffB1 = drive.trajectorySequenceBuilder(pickupB1.end())
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 10, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))
//                .lineToLinearHeading(new Pose2d(-55.80, -55.80, Math.toRadians(45.00)))
//                .build(); //run intake 10%

//raise 4200
//drop block

        TrajectorySequence pickupB2 = drive.trajectorySequenceBuilder(pickupB1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 25, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-59.00, -48.00, Math.toRadians(89.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(55))
                .lineToLinearHeading(new Pose2d(-59.00, -39.00, Math.toRadians(89.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 20, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-55.80, -55.80, Math.toRadians(45.00)))
                .build();//drop to 0

//grab block 2

//        TrajectorySequence dropoffB2 = drive.trajectorySequenceBuilder(pickupB2.end())
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 10, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
//                .lineToLinearHeading(new Pose2d(-55.80, -55.80, Math.toRadians(45.00)))
//                .build(); //run intake 10%

//raise 4200
//drop block

        TrajectorySequence pickupB3 = drive.trajectorySequenceBuilder(pickupB2.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-59.00, -48.00, Math.toRadians(124.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50, 15, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-59.00, -39.00, Math.toRadians(124.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 20, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-56.80, -55.20, Math.toRadians(45.00)))
                .build();

//grab block 3

//        TrajectorySequence dropoffB3 = drive.trajectorySequenceBuilder(pickupB3.end())
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, 10, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
//                .lineToLinearHeading(new Pose2d(-56.80, -55.20, Math.toRadians(45.00)))
//                .build(); //run intake 10%

//raise 4200
//drop block

        TrajectorySequence park = drive.trajectorySequenceBuilder(pickupB3.end())
                .lineToLinearHeading(new Pose2d(-34.00, -19.50, Math.toRadians(12.00)))
                .build();

//extend slide to touch bar


        drive.setPoseEstimate(drivesubhigh.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(drivesubhigh);

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

                }
                //drops the speicmen and drives to B1
            } else if (state == 2) {
                if (elevator.atTarget()) {
                    drive.followTrajectorySequenceAsync(pickupB1);
                    elevator.setHeight(0);
                    grabber.armToFloor();
                    grabber.intakeIn();
                    state++;
                }
            } else if (state == 3) {
                //intake B1
                    state++;

            } else if (state == 4) {
                state++;
                //drop the B1 block
            } else if (state == 5) {
                if (!drive.isBusy()) {
                    grabber.intakeStop();
                    grabber.armToInside();
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
                    grabber.armToFloor();
                    state++;
                }
            } else if (state == 8) {
                //drive to intake B2
                if (elevator.atTarget(1000)) {
                    grabber.intakeIn();
                    drive.followTrajectorySequenceAsync(pickupB2);
                    state++;

                }
            } else if (state == 9) {
                //intake B2
                state++;
            } else if (state == 10) {
                state++;
            } else if (state == 11) {
                //drop B2
                if (!drive.isBusy()) {
                    grabber.armToInside();
                    grabber.intakeStop();
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
                if (elevator.atTarget(2000)) {
                    grabber.armToFloor();
//                    grabber.slideToPercent(.27);
                    grabber.intakeIn();
                    drive.followTrajectorySequenceAsync(pickupB3);
                    state++;
                }
            } else if (state == 15) {
                //intake B3
                state++;
            } else if (state == 16) {
                state++;
            } else if (state == 17) {
                //drop B3
                if (!drive.isBusy()) {
                    elevator.setHeight(4150);
                    grabber.intakeStop();
                    grabber.armToInside();
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
                    grabber.armToHook();
                    elevator.setHeight(800);
                    state++;
                }
            } else if (state == 20) {
                //PARK
                if (elevator.atTarget(2500)) {
                    drive.followTrajectorySequenceAsync(park);
                    grabber.slideToPercent(.5);
                    timer.start();
                    state++;
                }
            } else if (state == 21) {
                if(!drive.isBusy()) {
                    grabber.slideToPercent(.90);
                }
            }
        }
    }
}




