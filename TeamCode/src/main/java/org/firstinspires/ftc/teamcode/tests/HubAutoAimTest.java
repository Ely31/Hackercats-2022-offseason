package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.PIDArmSystem;
import org.firstinspires.ftc.teamcode.hardware.TeleopDrive;
import org.firstinspires.ftc.teamcode.vision.HubAutoAimPipeline;

@Config
@TeleOp(group="test")
public class HubAutoAimTest extends LinearOpMode {
    // Pre-init
    Camera camera = new Camera();
    HubAutoAimPipeline autoAimPipeline = new HubAutoAimPipeline();

    TeleopDrive drive = new TeleopDrive();
    PIDArmSystem armSystem = new PIDArmSystem();

    ElapsedTime loopTimer;
    ElapsedTime turretAngleUpdateTimer;
    @Override
    public void runOpMode() {
        // Init
        camera.init(hardwareMap);
        camera.webcam.setPipeline(autoAimPipeline);
        FtcDashboard.getInstance().startCameraStream(camera.webcam, 5); // Stream to dashboard at 5 fps

        drive.init(hardwareMap);
        armSystem.init(hardwareMap);

        armSystem.setFourBarAngle(30);

        loopTimer = new ElapsedTime();
        turretAngleUpdateTimer = new ElapsedTime();
        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.dpad_left) autoAimPipeline.outputMode = false;
            if (gamepad1.dpad_right) autoAimPipeline.outputMode = true;

            drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            if (gamepad1.back) drive.resetHeading();

            if (turretAngleUpdateTimer.milliseconds() > 200) armSystem.turretTargetAngle -= (autoAimPipeline.getAngle());

            armSystem.setTurretAngle(armSystem.turretTargetAngle);
            armSystem.update();

            telemetry.addData("loop time", loopTimer.milliseconds());
            telemetry.addData("target angle", armSystem.turretTargetAngle);
            telemetry.addData("corrected x", autoAimPipeline.getCorrectedX());
            telemetry.addData("pipeline angle", autoAimPipeline.getAngle());
            telemetry.update();

            loopTimer.reset();
        }
    }
}
