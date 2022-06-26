package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.vision.AutoAimPipeline;

@TeleOp(group="test")
public class AutoAimTest extends LinearOpMode {
    // Pre-init
    Camera camera = new Camera();
    AutoAimPipeline autoAimPipeline = new AutoAimPipeline(telemetry);
    @Override
    public void runOpMode() {
        // Init
        camera.init(hardwareMap);
        camera.webcam.setPipeline(autoAimPipeline);

        FtcDashboard.getInstance().startCameraStream(camera.webcam, 5); // Stream to dashboard at 5 fps
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.dpad_left) autoAimPipeline.outputMode = false;
            if (gamepad1.dpad_right) autoAimPipeline.outputMode = true;
        }
    }
}
