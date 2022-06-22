package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(group = "test")
public class PIDTest extends LinearOpMode {
    // Pre-init
    final double COUNTS_PER_REV = 103.8;

    DcMotor carousel;
    public static PIDCoefficients coeffs = new PIDCoefficients(5,0,0.8);
    PIDFController controller = new PIDFController(coeffs);
    double targetPos = 1;
    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Graph stuff on dashboard
        carousel = hardwareMap.get(DcMotor.class,"test");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.a) targetPos = 1;
            if (gamepad1.b) targetPos = -1;

            controller.setTargetPosition(targetPos);

            carousel.setPower(controller.update(carousel.getCurrentPosition()/COUNTS_PER_REV));

            telemetry.addData("error", controller.getLastError());
            telemetry.addData("correction", controller.update(carousel.getCurrentPosition()/COUNTS_PER_REV));
            telemetry.addData("position", carousel.getCurrentPosition()/COUNTS_PER_REV);
            telemetry.update();
        }
    }
}
