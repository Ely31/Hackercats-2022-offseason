package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Intake;

@Autonomous(group="test")
public class IntakeSensorTest extends LinearOpMode {
    // Pre-init
    Intake intake = new Intake();
    @Override
    public void runOpMode() {
        // Init
        intake.init(hardwareMap);
        telemetry.setMsTransmissionInterval(100);
        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // Autonomous instructions
            telemetry.addData("freight status", intake.freightStatus());
            telemetry.addData("max proximity", intake.maxProximity());
            telemetry.update();
        }
    }
}
