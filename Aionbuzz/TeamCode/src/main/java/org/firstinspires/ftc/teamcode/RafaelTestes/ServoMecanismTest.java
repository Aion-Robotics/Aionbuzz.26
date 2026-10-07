package org.firstinspires.ftc.teamcode.RafaelTestes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class ServoMecanismTest extends OpMode {
    Servo servoPos = null;
    CRServo servoRot = null;

    public void init(){
        servoPos = hardwareMap.get(Servo.class, "Servo_Pos");
        servoRot = hardwareMap.get(CRServo.class,"ServoRot");
    }

    @Override
    public void loop() {

    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);

    }
    public void setServoRot(double power) {
        servoRot.setPower(power);
}}
