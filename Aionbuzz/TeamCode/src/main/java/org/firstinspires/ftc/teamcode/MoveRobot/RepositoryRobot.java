package org.firstinspires.ftc.teamcode.MoveRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RepositoryRobot extends OpMode {
    public void init(){
    }
    public void loop(){
    }
    public double distanceMeter(DcMotor[] motors){
        Double[] distance = new Double[2];
        for (int i = 0; i < 2; i++){
            int ticks = motors[i].getCurrentPosition();
            double cm = (ticks / 537.6) * (9.66 * Math.PI);
            distance[i] = cm;
        }
        return (distance[0] + distance[1]) / 2.0;
    }
}