package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.PIDArmSystem;
import org.firstinspires.ftc.teamcode.hardware.TeleopDrive;
import org.firstinspires.ftc.teamcode.vision.AutoAimPipeline;

@Config
@TeleOp(group="test")
public class AutoAimTest extends LinearOpMode {
    // Pre-init
    Camera camera = new Camera();
    AutoAimPipeline autoAimPipeline = new AutoAimPipeline();

    TeleopDrive drive = new TeleopDrive();
    PIDArmSystem armSystem = new PIDArmSystem();

    double targetAngle = 0;
    public static double p = 0.01; // P component in the P controller for the turret

    ElapsedTime looptimer;
    @Override
    public void runOpMode() {
        // Init
        camera.init(hardwareMap);
        camera.webcam.setPipeline(autoAimPipeline);
        FtcDashboard.getInstance().startCameraStream(camera.webcam, 5); // Stream to dashboard at 5 fps

        drive.init(hardwareMap);
        armSystem.init(hardwareMap);

        armSystem.setFourBarAngle(30);

        looptimer = new ElapsedTime();
        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.dpad_left) autoAimPipeline.outputMode = false;
            if (gamepad1.dpad_right) autoAimPipeline.outputMode = true;

            drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);

            targetAngle -= (autoAimPipeline.getCorrectedX() * p);

            armSystem.setTurretAngle(targetAngle);
            armSystem.update();

            telemetry.addData("loop time", looptimer.milliseconds());
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("corrected x", autoAimPipeline.getCorrectedX());
            telemetry.update();

            looptimer.reset();
        }
    }
}
