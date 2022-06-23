package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.CapMech;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.vision.SkystoneStyleThreshold;

@Config
@Autonomous
public class CyclesAutoBeta extends LinearOpMode {
    // Pre-init
    Camera webcam  = new Camera();
    SkystoneStyleThreshold pipeline = new SkystoneStyleThreshold();
    SampleMecanumDriveCancelable drive;
    ArmSystem armSystem = new ArmSystem();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
    CapMech capMech = new CapMech();

    int hubActiveLevel = 0;

    int side; // Red alliance is 1, blue is -1
    boolean deepPark = true;

    final double originToWall = 141.0/2.0; // I guess the field is actually 141 inches wide
    final double wallDistance = originToWall - 6.5; // Center of bot is 6.5in from wall

    // Realative to warehouse
    private Pose2d farTsePosition;
    private Pose2d middleTsePosition;
    private Pose2d closeTsePosition;
    private Pose2d closeTsePosPreMove;

    Pose2d startPos = new Pose2d(11.4,-(originToWall-9), Math.toRadians(-90));
    Pose2d depositPos;
    Pose2d tsePos;
    TrajectorySequence pickUpTSE;
    Trajectory depositPreLoad;
    TrajectorySequence intoWarehouse;
    Trajectory approachFreight;
    TrajectorySequence outAndDeposit;
    TrajectorySequence park;

    public static boolean extraCycle = true;
    ElapsedTime approachFreightTimeout = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Init
        webcam.init(hardwareMap);
        webcam.webcam.setPipeline(pipeline);
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(startPos);
        armSystem.init(hardwareMap);
        armSystem.setArmPosition(0,0);
        deposit.init(hardwareMap);
        intake.init(hardwareMap);
        capMech.init(hardwareMap);

        ElapsedTime depositTimer = new ElapsedTime();
        ElapsedTime pipelineThrottle = new ElapsedTime();

        FtcDashboard.getInstance().startCameraStream(webcam.webcam, 1); // Stream to dashboard at 1 fps

        AutoToTele.allianceSide = 1;

        tsePos = new Pose2d(12,-50*side,Math.toRadians(225*side));

        while (!isStarted()&&!isStopRequested()){ // Init loop

            // Select alliance with gamepad and display it to telemetry
            if (gamepad1.b) AutoToTele.allianceSide = 1;
            if (gamepad1.x) AutoToTele.allianceSide = -1;

            if (gamepad1.dpad_up) deepPark = true;
            if (gamepad1.dpad_down) deepPark = false;

            side = AutoToTele.allianceSide;
            switch (side) {
                case 1:
                    telemetry.addLine("red alliance");
                    break;
                case -1:
                    telemetry.addLine("blue alliance");
                    break;
            }
            telemetry.addData("going to level", hubActiveLevel);
            telemetry.addData("deepPark", deepPark);
            telemetry.addData("freight status", intake.freightStatus());
            telemetry.update();

           if (pipelineThrottle.milliseconds() > 1000) {// Throttle loop times to 1 second
               // Update startpos to match side
               startPos = new Pose2d(11.4,(-(originToWall-9))*side, Math.toRadians(-90*side));
               drive.setPoseEstimate(startPos); // Set pose estimate to match which side of the field we're on

               farTsePosition = new Pose2d(2.5,-48*side,Math.toRadians(-90*side));
               middleTsePosition = new Pose2d(11.8,-48*side,Math.toRadians(-90*side));
               closeTsePosition = new Pose2d(9.5,-44*side,Math.toRadians(225*side));

               closeTsePosPreMove = new Pose2d(4,-48*side,tsePos.getHeading());

               switch (pipeline.getAnalysis()){
                   case LEFT:
                       hubActiveLevel = 1;
                       break;
                   case MIDDLE:
                       hubActiveLevel = 2;
                       break;
                   case RIGHT:
                       hubActiveLevel = 3;
                       break;
               }

               switch (hubActiveLevel) {
                   case 1:
                       depositPos = new Pose2d(-2, -41*side, Math.toRadians(-70*side));
                       if (side == 1){ // Switch close and far positions on blue alliance
                           tsePos = farTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(new Pose2d(tsePos.getX(),-55*side,tsePos.getHeading())) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       else {
                           tsePos = closeTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(closeTsePosPreMove) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       break;
                   case 2:
                       depositPos = new Pose2d(-2, -41*side, Math.toRadians(-70*side));
                       tsePos = middleTsePosition;
                       pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                               .lineToSplineHeading(tsePos)
                               .build();
                       break;
                   case 3:
                       depositPos = new Pose2d(-2, -41*side, Math.toRadians(-70*side));
                       if (side == 1){  // Switch close and far positions on blue alliance
                           tsePos = closeTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(closeTsePosPreMove) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       else {
                           tsePos = farTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(new Pose2d(tsePos.getX(),-55*side,tsePos.getHeading())) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       break;
               }

               // Deposit trajectory
               depositPreLoad = drive.trajectoryBuilder(pickUpTSE.end())
                       .lineToSplineHeading(depositPos,
                               SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                               SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                       .build();

               // Cycle trajectory
               intoWarehouse = drive.trajectorySequenceBuilder(depositPreLoad.end())
                       // Retract arm while driving away
                       .addTemporalMarker(0.5, () -> armSystem.retract())
                       // Spline into warehouse
                       .lineToSplineHeading(new Pose2d(2,(-55*side), Math.toRadians(0*side)))
                       .splineToConstantHeading(new Vector2d(30,(-63*side)),Math.toRadians(0*side))
                       .addTemporalMarker(()-> intake.on())
                       .lineTo(new Vector2d(36.5,-63*side)) // Go into warehouse
                       .build();

               approachFreight = drive.trajectoryBuilder(intoWarehouse.end())
                       // Creep forward slowly
                       .forward(20,
                               SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                               SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                       .build();

               // Park trajectory
               if (deepPark) park = drive.trajectorySequenceBuilder(depositPreLoad.end())
                       // Lower arm while driving
                       .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                       .addTemporalMarker(0.7, () -> armSystem.setArmPosition(0,0))
                       .addTemporalMarker(0.5, () -> armSystem.setArmPosition(0,0))
                       // Back off hub
                       .lineToSplineHeading(new Pose2d(2,(-55*side), Math.toRadians(0*side)))
                       // Spline into warehouse
                       .splineToConstantHeading(new Vector2d(29,(-63*side)),Math.toRadians(0*side))
                       // Move toward shared hub a bit slower to prevent slipping
                       .splineToConstantHeading(new Vector2d(37,(-44*side)), Math.toRadians(90*side),
                               SampleMecanumDriveCancelable.getVelocityConstraint(30,Math.toRadians(120), 11),
                               SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                       // Park
                       .splineToSplineHeading(new Pose2d(63,(-37*side), Math.toRadians(-90*side)), Math.toRadians(0*side))
                       .build();
               else {
                   park = drive.trajectorySequenceBuilder(depositPreLoad.end())
                           .addTemporalMarker(0.7, () -> armSystem.retract())
                           .addTemporalMarker(0.5, () -> armSystem.setArmPosition(0, 0))
                           // Back off hub
                           .lineToSplineHeading(new Pose2d(2,(-55*side), Math.toRadians(0*side)))
                           // Slip into warehouse
                           .splineToConstantHeading(new Vector2d(36,(-63*side)),Math.toRadians(0*side))
                           .build();
               }
               // Telemetry
               telemetry.addData("going to level", hubActiveLevel);
               telemetry.update();
               pipelineThrottle.reset(); // Reset the throttle timer so the whole thing loops
           } // End of throttled section
        }// End of init loop

        waitForStart();
        // Pre-run
    
        if (opModeIsActive()) {
            // Autonomous instructions
            capMech.openGripper();
            capMech.levelArm();
            drive.followTrajectorySequence(pickUpTSE); // Drive to tse
            capMech.closeGripper();
            sleep(250); // Wait for gripper to close
            capMech.retract();
            armSystem.runToLevel(hubActiveLevel); // Extend 4b before driving
            drive.followTrajectory(depositPreLoad); // Drive to spot where we'll deposit from
            depositTimer.reset();
            deposit.dump(depositTimer); // Dump
            sleep(310); // Wait for that dump to finish
            deposit.dump(depositTimer);
            intake.dropIntake();

            drive.followTrajectorySequence(intoWarehouse);

            drive.followTrajectoryAsync(approachFreight); // Follow async
            approachFreightTimeout.reset();
            // Stop the trajectory if it hasn't found a freight in 3 seconds
            while (intake.isEmpty() && (approachFreightTimeout.milliseconds() < 3000)){
                drive.update();
            }
            // Copied from the lrr cancel trajectory sample
            drive.breakFollowing();
            drive.setDrivePower(new Pose2d());
            // After we detect a freight and the traj ends, go out and score it
            outAndDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .waitSeconds(0.5)
                    .addTemporalMarker(()->intake.off())
                    .addTemporalMarker(0.8,()-> intake.on())
                    .lineTo(new Vector2d(10,-63*side)) // Go out of warehouse
                    .addTemporalMarker(()-> armSystem.runToLevel(3))
                    .addTemporalMarker(()-> intake.off())
                    // Go to shipping hub
                    .splineToSplineHeading(new Pose2d(-2, -42*side, Math.toRadians(-60*side)),Math.toRadians(120*side),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(()->{ // Dump it
                        depositTimer.reset();
                        deposit.dump(depositTimer);
                    })
                    .waitSeconds(0.31)
                    .addTemporalMarker(()->deposit.dump(depositTimer))
                    .build();

            drive.followTrajectorySequence(outAndDeposit);

            if (extraCycle) {
                drive.followTrajectorySequence(intoWarehouse);

                // Update approach freight
                approachFreight = drive.trajectoryBuilder(intoWarehouse.end())
                        // Creep forward slowly
                        .forward(20,
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                drive.followTrajectoryAsync(approachFreight); // Follow async
                approachFreightTimeout.reset();
                while (intake.isEmpty() &&(approachFreightTimeout.milliseconds() < 3000)){
                    drive.update();
                }
                // Copied from the lrr cancel trajectory sample
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                // After we detect a freight and the traj ends, go out and score it
                outAndDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(0.5)
                        .addTemporalMarker(()->intake.off())
                        .addTemporalMarker(0.8,()-> intake.on())
                        .lineTo(new Vector2d(10,-63*side)) // Go out of warehouse
                        .addTemporalMarker(()-> armSystem.runToLevel(3))
                        .addTemporalMarker(()-> intake.off())
                        // Go to shipping hub
                        .splineToSplineHeading(new Pose2d(-2, -42*side, Math.toRadians(-60*side)),Math.toRadians(120*side),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addTemporalMarker(()->{ // Dump it
                            depositTimer.reset();
                            deposit.dump(depositTimer);
                        })
                        .waitSeconds(0.31)
                        .addTemporalMarker(()->deposit.dump(depositTimer))
                        .build();

                drive.followTrajectorySequence(outAndDeposit);
            }

            drive.followTrajectorySequence(park); // Park in warehouse

            // Save this information to a class so we can use it in tele to calibate feild centric
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();
        }
    }
}
