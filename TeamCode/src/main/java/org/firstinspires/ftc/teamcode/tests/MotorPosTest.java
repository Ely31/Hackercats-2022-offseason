package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorPosTest",group="test")
public class MotorPosTest extends LinearOpMode {
    // Pre-init
    DcMotor test;

    double TICKS_PER_DEGREE = (1425.1 * 1.5625)/360;
    @Override
    public void runOpMode() {
        // Init
        test = hardwareMap.dcMotor.get("test");
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            telemetry.addData("pos", test.getCurrentPosition()/TICKS_PER_DEGREE);
            telemetry.update();
        }
    }
}
