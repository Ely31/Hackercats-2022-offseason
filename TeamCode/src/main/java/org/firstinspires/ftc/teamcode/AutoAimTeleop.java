package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.CapMech;
import org.firstinspires.ftc.teamcode.hardware.CarouselMech;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PIDArmSystem;
import org.firstinspires.ftc.teamcode.hardware.TeleopDrive;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TruePress;
import org.firstinspires.ftc.teamcode.vision.HubAutoAimPipeline;

@Config
@TeleOp
public class AutoAimTeleop extends LinearOpMode {
    // Pre-init

    TeleopDrive drive = new TeleopDrive();
    Intake intake = new Intake();
    Deposit deposit = new Deposit();
    PIDArmSystem pidArmSystem = new PIDArmSystem();
    CarouselMech carouselMech = new CarouselMech();
    CapMech capMech = new CapMech();
    Camera camera = new Camera();

    HubAutoAimPipeline pipeline = new HubAutoAimPipeline();

    TruePress fourbarToggleInput = new TruePress();
    TruePress capMechToggleInput = new TruePress();
    TruePress autoAimToggleInput = new TruePress();

    ElapsedTime dumptime = new ElapsedTime(300);

    enum FourBarState {
        EXTENDED,
        RETRACTED
    }
    FourBarState fourBarState = FourBarState.RETRACTED;

    boolean autoAim = false;
    public static double autoAim_P = 0.1;

    enum CarouselState {
        RUNNING,
        STOPPED
    }
    AutoAimTeleop.CarouselState carouselState = AutoAimTeleop.CarouselState.STOPPED;
    ElapsedTime carouselAccelTimer = new ElapsedTime();
    public static double carouselAccelRate = 0.015;

    boolean capMechState = false; // True means extended, false means retracted

    int side = AutoToTele.allianceSide;

    public static boolean debug = false;

    @Override
    public void runOpMode() {
        // Init
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        deposit.init(hardwareMap);
        pidArmSystem.init(hardwareMap);
        pidArmSystem.retract();
        carouselMech.init(hardwareMap);
        capMech.init(hardwareMap);
        // Camera setup for autoaim
        camera.init(hardwareMap);
        camera.webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(camera.webcam, 5);

        // Enable bulk reads
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        sleep(100); // Hack to fix deposit firing if you init too quickly

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Send stuff to dashboard to graph
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        carouselAccelTimer.reset();
        // Pre-run
        while (opModeIsActive()) { // TeleOp loop
            // Mecdrive control
            drive.driveFieldCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_trigger);
            if (gamepad1.back) drive.resetHeading(); // Reset field centric if needed

            // Intake control
            if (gamepad1.b) intake.reverse();
            else if (fourBarState == FourBarState.EXTENDED) intake.off();
            else intake.toggle(gamepad1.a);
            if (gamepad2.back) intake.dropIntake(); // In case this doesn't happen in auto for some reason

            // Deposit control
            if (gamepad1.left_bumper && !capMechState) dumptime.reset();
            deposit.dump(dumptime);

            // Change active level of 4b
            if (gamepad2.x) pidArmSystem.activeLevel = 1;
            else if (gamepad2.y) pidArmSystem.activeLevel = 2;
            else if (gamepad2.b) pidArmSystem.activeLevel = 3;

            // Tune the height of the active 4b level to account for shipping hub inbalance
            if (gamepad2.dpad_up) pidArmSystem.editFourbarLevelOffset(0.4); // This number is small because it's added every loop
            if (gamepad2.dpad_down) pidArmSystem.editFourbarLevelOffset(-0.4);

            if (autoAimToggleInput.trueInput(gamepad2.back)) autoAim = !autoAim; // Toggle auto aim

            // Four bar control
            switch (fourBarState) { // Gamepad2 A toggles the extended/retracted state of the 4b
                case RETRACTED:
                    pidArmSystem.retract(); // Retract arm
                    // By default bring the turret to it's last position when extending,
                    // press the left stick to reset it to zero so it doesn't move when you extend
                    if (gamepad2.left_stick_button) pidArmSystem.turretTargetAngle = 0;
                    if (autoAim) pidArmSystem.turretTargetAngle = -20 * side;

                    if (fourbarToggleInput.trueInput(gamepad2.a)) { // Use A to switch states
                    fourBarState = FourBarState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    // Run the 4b and turret to their desired positions
                    pidArmSystem.setArmPosition(pidArmSystem.levelToAngle(pidArmSystem.activeLevel), pidArmSystem.turretTargetAngle);
                    if (autoAim) {
                        pidArmSystem.turretTargetAngle += (pipeline.getAngle() * autoAim_P);
                    }
                    else pidArmSystem.turretTargetAngle += (gamepad2.left_trigger - gamepad2.right_trigger) * 2.5;

                    if (gamepad2.left_stick_button) pidArmSystem.turretTargetAngle = 0;

                    if (fourbarToggleInput.trueInput(gamepad2.a)) { // Use A to switch states
                        fourBarState = FourBarState.RETRACTED;
                    }
                    break;
            }
            pidArmSystem.update(); // Call this every loop to run the pid controls

            // Cap mech control
            if (capMechToggleInput.trueInput(gamepad2.dpad_left)) capMechState = !capMechState; // Raise and lower with dpad left
            if (capMechState) capMech.levelArm();
            else capMech.retract();

            if (capMechState && gamepad2.dpad_right) capMech.openGripper(); // Only let the gripper open when the capmech is down
            else capMech.closeGripper();

            // Carousel mech control
            // If you're pressing a bumper, set the state to running
            // If not, set it to stopped
            if (gamepad2.left_bumper || gamepad2.right_bumper) carouselState = AutoAimTeleop.CarouselState.RUNNING;
            else carouselState = AutoAimTeleop.CarouselState.STOPPED;

            // Fsm and non blocking timers go brrr
            switch (carouselState){
                case RUNNING:
                    if (carouselAccelTimer.milliseconds() > 100) { // Increase the max speed every 100ms
                        carouselMech.MAX_SPEED += carouselAccelRate;
                        carouselAccelTimer.reset();
                    }
                    if (gamepad2.left_bumper) carouselMech.setSpeed(1 * side); // The setSpeed method multiplies input by the max speed
                    else if (gamepad2.right_bumper) carouselMech.setSpeed(-1 * side);
                    else carouselMech.setSpeed(0);
                    break;
                case STOPPED:
                    carouselMech.MAX_SPEED = 0.20; // Reset max speed to normal after the trigger is released
                    carouselMech.setSpeed(0);
                    carouselAccelTimer.reset();
            }

            telemetry.addData("autoaim enabled", autoAim);

            if (debug) { // Send data to telemetry for debug purposes if we want to
                telemetry.addData("4b state", fourBarState);
                telemetry.addData("4b pos", pidArmSystem.FourbarAngle());
                telemetry.addData("4b target angle", pidArmSystem.levelToAngle(pidArmSystem.activeLevel));
                telemetry.addData("4b error", pidArmSystem.liftController.getLastError());
                telemetry.addData("turretpos", pidArmSystem.TurretAngle());
                telemetry.addData("turret target", pidArmSystem.turretTargetAngle);
                telemetry.addData("turret error", pidArmSystem.turretController.getLastError());
            }
            telemetry.update();
        }
    }
}
