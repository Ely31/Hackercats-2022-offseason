package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Deposit {

    private Servo pusher;
    private Servo door;

    public static double PUSHER_HOME_POSITION = 0.05; // The retracted position of the pusher
    double DOOR_HOME_POSITION = 0.15; // The closed position of the door
    public static double PUSHER_PUSHING_POSITION = 0.45;
    double DOOR_OPEN_POSITION = 0.65;

    //time to wait before we close the door and reset
    // the pusher after the freight has fallen out
    private final int FALL_TIME = 300;

    public void init(HardwareMap hwmap) {
        pusher = hwmap.get(Servo.class, "pusher");
        door = hwmap.get(Servo.class, "door");

        pusher.setPosition(PUSHER_HOME_POSITION);
        door.setPosition(DOOR_HOME_POSITION);
    }

    public void dump(ElapsedTime time){
        if (time.milliseconds() > 0 && time.milliseconds() < FALL_TIME){
            doorOpen();
            push();// Open the door if the timer is less than pusherlag
        }
        if (time.milliseconds() > FALL_TIME) reset(); // Reset deposit after falltime is over
    }

    public void reset(){
        pusher.setPosition(PUSHER_HOME_POSITION);
        door.setPosition(DOOR_HOME_POSITION);
    }
    public void doorOpen(){
        door.setPosition(DOOR_OPEN_POSITION);
    }
    public void push(){
        pusher.setPosition(PUSHER_PUSHING_POSITION);
    }
}
