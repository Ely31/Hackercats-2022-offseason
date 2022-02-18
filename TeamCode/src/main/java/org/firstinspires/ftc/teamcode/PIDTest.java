package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class PIDTest extends LinearOpMode {
    // Pre-init
    final double COUNTS_PER_REV = 103.8;

    DcMotor test;
    PIDCoefficients coeffs = new PIDCoefficients(100,0,0);
    PIDFController controller = new PIDFController(coeffs);
    double targetPos = 1;
    double correction;
    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Graph stuff on dashboard
        test = hardwareMap.get(DcMotor.class,"test");
        test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.a) targetPos = 1;
            if (gamepad1.b) targetPos = -1;

            controller.setTargetPosition(targetPos);
            correction = controller.update(test.getCurrentPosition()/COUNTS_PER_REV);

            test.setPower(correction);

            telemetry.addData("error", controller.getLastError());
            telemetry.addData("correction", correction);
            telemetry.addData("position", test.getCurrentPosition()/COUNTS_PER_REV);
            telemetry.update();
        }
    }
}
