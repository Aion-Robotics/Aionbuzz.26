package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MoveRobot.RepositoryRobot;


@TeleOp
public class TankDrive extends RepositoryRobot {
    @Override
    public void init(){
        initializerHardware();
        telemetry.addData("Hardware Status", "Registered");
    }

    @Override
    public void loop(){
        // Odometry
        double distance = distanceMeter();
        telemetry.addData("Distance", "%.1f cm", distance);

        moveRobot();
        energyIntake();
        servoCR();
    }
}
