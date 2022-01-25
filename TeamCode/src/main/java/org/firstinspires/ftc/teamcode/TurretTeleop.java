package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.hardware.CapMech;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.TeleopDrive;
import org.firstinspires.ftc.teamcode.util.TruePress;

@Config
@TeleOp
public class TurretTeleop extends LinearOpMode {
    // Pre-init

    TeleopDrive drive = new TeleopDrive();
    Intake intake = new Intake();
    Deposit deposit = new Deposit();
    ArmSystem armSystem = new ArmSystem();
    CarouselSpinner carouselSpinner = new CarouselSpinner();
    CapMech capMech = new CapMech();

    TruePress fourbarToggleInput = new TruePress();
    TruePress capMechToggleInput = new TruePress();

    ElapsedTime dumptime = new ElapsedTime();

    enum FourBarState {
        EXTENDED,
        RETRACTED
    }

    FourBarState fourBarState = FourBarState.RETRACTED;

    boolean capMechState = false; // True means extended, false means retracted

    public static boolean debug = false;

    @Override
    public void runOpMode() {
        // Init
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        deposit.init(hardwareMap);
        armSystem.init(hardwareMap);
        armSystem.setArmPosition(0,0);
        carouselSpinner.init(hardwareMap);
        capMech.init(hardwareMap);

        dumptime.reset();
        sleep(200); // Hack to fix deposit firing if you init too quickly

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        // Pre-run
        while (opModeIsActive()) { // TeleOp loop
            // Mecdrive control
            drive.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_trigger);
            if (gamepad1.back) drive.resetHeading(); // Reset field centric if needed

            // Intake control
            if (gamepad1.b) intake.reverse();
            else if (fourBarState == FourBarState.EXTENDED) intake.off();
            else intake.toggle(gamepad1.a);
            if (gamepad2.back) intake.dropIntake(); // In case this doesn't happen in auto for some reason

            // Deposit control
            if (gamepad1.left_bumper) dumptime.reset();
            deposit.dump(dumptime);

            // Change active level of 4b
            if (gamepad2.x) armSystem.activeLevel = 1;
            else if (gamepad2.y) armSystem.activeLevel = 2;
            else if (gamepad2.b) armSystem.activeLevel = 3;

            // Tune the height of the active 4b level to account for shipping hub inbalance
            if (gamepad2.dpad_up) armSystem.editFourbarLevelOffset(0.3); // This number is small because it's added every loop
            if (gamepad2.dpad_down) armSystem.editFourbarLevelOffset(-0.3);

            // Four bar control
            switch (fourBarState) { // Gamepad2 A toggles the extended/retracted state of the 4b
                case RETRACTED:
                    armSystem.setArmPosition(0,0); // Retract arm
                    // Reset turret angle so that when the arm is first extended the turret doesn't move
                    armSystem.turretTargetAngle = 0;

                    if (fourbarToggleInput.trueInput(gamepad2.a)) { // Use A to switch states
                    fourBarState = FourBarState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    // Run the 4b and turret to their desired positions
                    armSystem.setArmPosition(armSystem.levelToAngle(armSystem.activeLevel), armSystem.turretTargetAngle);
                    if (gamepad2.left_bumper) armSystem.turretTargetAngle += 2; // Change turret angle with bumpers
                    if (gamepad2.right_bumper) armSystem.turretTargetAngle -= 2;// Change turret angle with bumpers

                    if (fourbarToggleInput.trueInput(gamepad2.a)) { // Use A to switch states
                        fourBarState = FourBarState.RETRACTED;
                    }
                    break;
            }

            // Cap mech control
            if (capMechToggleInput.trueInput(gamepad2.dpad_left)) capMechState = !capMechState; // Raise and lower with dpad left
            if (capMechState) capMech.levelArm();
            else capMech.retract();

            if (capMechState && gamepad2.dpad_right) capMech.openGripper(); // Only let the gripper open when the capmech is down
            else capMech.closeGripper();

            // Carousel mech control
            carouselSpinner.setSpeed(gamepad2.left_trigger- gamepad2.right_trigger);
            // Subracting the right trigger input from the left is an easy way to make the left trigger
            // turn it one way and the right trigger turn it the opposite way

            if (debug) { // Send data to telemetry for debug purposes if we want to
                telemetry.addData("4b state", fourBarState);
                telemetry.addData("4b pos", armSystem.FourbarAngle());
                telemetry.addData("target level", armSystem.activeLevel);
                telemetry.addData("turretpos", armSystem.TurretAngle());
                telemetry.addData("turret target", armSystem.turretTargetAngle);
                telemetry.addData("heading", -drive.heading);
                telemetry.addData("proximity", intake.sensorProximity());
                telemetry.addData("intaken", intake.freightStatus());
                telemetry.update();
            }
        }
    }
}
