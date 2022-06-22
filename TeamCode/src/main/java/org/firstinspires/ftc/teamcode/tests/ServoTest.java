package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ServoTest",group="test")
public class ServoTest extends LinearOpMode {
    // Pre-init
    public static double Pos1 = 0;
    public static double Pos2 = 1;
    public static double PosCenter = (Pos1+Pos2/2);

    Servo test;

    @Override
    public void runOpMode() {
        // Init
    test = hardwareMap.servo.get("test");
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.x) test.setPosition(Pos1);
            if (gamepad1.b) test.setPosition(Pos2);
            if (gamepad1.a) test.setPosition(PosCenter);

            PosCenter = (Pos1+Pos2/2);

            telemetry.addData("pos",test.getPosition());
            telemetry.update();
        }
    }
}
