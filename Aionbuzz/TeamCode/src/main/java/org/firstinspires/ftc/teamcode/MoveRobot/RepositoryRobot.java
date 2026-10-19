package org.firstinspires.ftc.teamcode.MoveRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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
    public void moveRobot(DcMotor[] motor){
        DcMotor.ZeroPowerBehavior stateMotor;
        double stickY =  gamepad1.left_stick_y;
        double stickX =  gamepad1.left_stick_x;

        double leftMove = Range.clip(stickX - stickY,-0.5, 0.5);
        double rightMove = Range.clip(stickX + stickY, -0.5, 0.5);

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
    public void energyIntake(DcMotor[] motor){
        if (gamepad1.left_trigger_pressed) {
            motor[2].setPower(1.0);
            telemetry.addData("Intake Status", "Working");
        }
        else
            motor[2].setPower(0);
    }
    public void servoCR(CRServo[] speedServo) {
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