package org.firstinspires.ftc.teamcode.Mecanismos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter3 extends OpMode {

    private DcMotorEx motorShooter;

    private static final String MOTOR_NAME = "motorShooter";
    private static final double MAX_RPM = 5990.4;

    public void init(){
        motorShooter = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    @Override
    public void loop(){}
}
