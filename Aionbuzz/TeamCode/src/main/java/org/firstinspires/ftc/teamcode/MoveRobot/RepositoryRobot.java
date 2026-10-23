package org.firstinspires.ftc.teamcode.MoveRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class RepositoryRobot extends OpMode {
    DcMotor[] motor = new DcMotor[3];
    CRServo[] speedServo = new CRServo[2];

    public void init(){
    }
    public void loop(){
    }
    public void initializerHardware(){
        // Program Motors
        motor[0] = hardwareMap.get(DcMotor.class, "motorDir");
        motor[1] = hardwareMap.get(DcMotor.class, "motorEsq");
        motor[2] = hardwareMap.get(DcMotor.class, "motorIntake");

        motor[1].setDirection(DcMotorSimple.Direction.REVERSE);

        // Program Servos
        speedServo[0] = hardwareMap.get(CRServo.class, "servoDir");
        speedServo[1] = hardwareMap.get(CRServo.class, "servoEsq");
        telemetry.addData("Servo Status", "Registered");

        speedServo[0].setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double distanceMeter(){
        Double[] distance = new Double[2];
        for (int i = 0; i < 2; i++){
            int ticks = motor[i].getCurrentPosition();
            double cm = (ticks / 537.6) * (9.66 * Math.PI);
            distance[i] = cm;
        }
        return (distance[0] + distance[1]) / 2.0;
    }
    public void moveRobot(){
        DcMotor.ZeroPowerBehavior stateMotor;
        double stickY =  -gamepad1.left_stick_y;
        double stickX =  gamepad1.left_stick_x;

        double leftMove = Range.clip(stickY - stickX,-0.5, 0.5);
        double rightMove = Range.clip(stickY + stickX, -0.5, 0.5);

        motor[0].setPower(rightMove);
        motor[1].setPower(leftMove);

        if (gamepad1.right_bumper) {
            double turbo = 2.0;
            for(int i = 0; i < 2; i++)
                motor[i].setPower(motor[i].getPower()*turbo);

        }
        if((leftMove == 0.0) && (rightMove == 0.0))
            stateMotor = DcMotor.ZeroPowerBehavior.BRAKE;
        else
            stateMotor = DcMotor.ZeroPowerBehavior.FLOAT;

        for (int i = 0; i < 2; i++)
            motor[i].setZeroPowerBehavior(stateMotor);
    }
    public void energyIntake(){
        if (gamepad1.left_trigger_pressed) {
            motor[2].setPower(1.0);
            telemetry.addData("Intake Status", "Working");
        }
        else
            motor[2].setPower(0);
    }
    public void servoCR() {
        if (gamepad1.left_bumper) {
            for (int i = 0; i < 2; i++)
                speedServo[i].setPower(1.0);
        } else {
            for (CRServo servo: speedServo) {
                servo.setPower(0.0);
            }
        }
    }
}