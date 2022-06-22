package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Intake {

    private DcMotor intake;
    private Servo intakeRelease;
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;

    double RELEASE_HOLD_POSITION = 0.1;
    double RELEASE_DROP_POSITION = 0.25;

    private boolean lastInput;
    private boolean intakeToggledStatus;

    public void init(HardwareMap hwmap) {
        intake = hwmap.get(DcMotor.class, "intake");
        intakeRelease = hwmap.get(Servo.class,"intakeRelease");
        leftSensor = hwmap.get(ColorSensor.class,"leftSensor");
        rightSensor = hwmap.get(ColorSensor.class,"rightSensor");
        resetDropper();
        lastInput = false;
        intakeToggledStatus = false;
    }

    public void on(){
        intake.setPower(1);
    }
    public void off(){
        intake.setPower(0);
    }
    public void reverse(){
        intake.setPower(-1);
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void toggle(boolean input){
        if (input && !lastInput){
            intakeToggledStatus = !intakeToggledStatus;
        }
        if (intakeToggledStatus) on();
        else off();

        lastInput = input;
    }

    public void dropIntake(){
        intakeRelease.setPosition(RELEASE_DROP_POSITION);
    }
    public void resetDropper(){
        intakeRelease.setPosition(RELEASE_HOLD_POSITION);
    }

    // Take the average of the values of both sensors
    public double maxProximity(){
        return Math.max(leftSensor.alpha(), rightSensor.alpha());
    }

    public boolean freightStatus(){
        double maxProximity = maxProximity(); // So we don't make two i2c calls
        return !(325 < maxProximity && maxProximity < 450);
    }

    public boolean isEmpty(){
        double maxProximity = maxProximity(); // So we don't make two i2c calls
        return (325 < maxProximity && maxProximity < 380);
    }
}
