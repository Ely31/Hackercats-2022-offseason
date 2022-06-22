package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest",group="test")
public class MotorTest extends LinearOpMode {
    // Pre-init
    DcMotor test;
    @Override
    public void runOpMode() {
        // Init
        test = hardwareMap.dcMotor.get("test");
        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            test.setPower(gamepad1.left_stick_y);

            if (gamepad1.a){
                test.setTargetPosition(0);
                test.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.b){
                test.setTargetPosition(1000);
                test.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}
