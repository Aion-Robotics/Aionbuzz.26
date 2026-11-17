package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class TankDrive extends RepositoryRobot {
    @Override
    public void init(){
        initializerHardware();
        telemetry.addData("Hardware Status", "Registered");
    }

    @Override
    public void loop(){
        // Odometria
        double distance = distanceMeter();
        telemetry.addData("Distance", "%.1f cm", distance);

        moveRobot();
        energyIntake();
        servoCR();
    }
}
