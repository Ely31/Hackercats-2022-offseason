package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class rrTest extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence test;
    Pose2d startpos = new Pose2d(-35,-(originToWall-9),Math.toRadians(0));

    static double originToWall = 141.0/2.0; // I guess the field is actually 141 inches wide
    static double wallDistance = originToWall - 6.5; // Center of bot is 6.5in from wall


    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startpos);

        test =  drive.trajectorySequenceBuilder(startpos)
                .lineToSplineHeading(new Pose2d(-7.0, -45,Math.toRadians(-70)))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(0,-wallDistance,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(40,-wallDistance, Math.toRadians(0)))
                .strafeLeft(26)
                .forward(18)
                .build();

        waitForStart();
        if (opModeIsActive()) {
            drive.followTrajectorySequence(test);
        }
    }
}
