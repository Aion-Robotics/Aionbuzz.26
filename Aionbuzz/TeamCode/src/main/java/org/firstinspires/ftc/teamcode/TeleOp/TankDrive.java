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
    DcMotor[] motor = new DcMotor[3];
    CRServo[] speedServo = new CRServo[2];


    @Override
    public void init(){
        // Program Motors
        motor[0] = hardwareMap.get(DcMotor.class, "motorDir");
        motor[1] = hardwareMap.get(DcMotor.class, "motorEsq");
        motor[2] = hardwareMap.get(DcMotor.class, "motorIntake");
        telemetry.addData("Motor Status", "Registered");

        // Program Servos
        speedServo[0] = hardwareMap.get(CRServo.class, "servoDir");
        speedServo[1] = hardwareMap.get(CRServo.class, "servoEsq");
        telemetry.addData("Servo Status", "Registered");

        speedServo[0].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        // Odometry
        double distance = distanceMeter(motor);
        telemetry.addData("Distance", "%.1f cm", distance);

        moveRobot(motor);
        energyIntake(motor);
        servoCR(speedServo);
    }
}
