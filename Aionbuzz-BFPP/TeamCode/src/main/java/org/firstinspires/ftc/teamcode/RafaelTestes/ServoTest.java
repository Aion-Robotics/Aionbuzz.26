package org.firstinspires.ftc.teamcode.RafaelTestes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class ServoTest extends OpMode {
    ServoMecanismTest servo = new ServoMecanismTest();
    double servoPos = 0.5;
    @Override
    public void init() {
        servo(hardwareMap);
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y;
    }

    private void servo(HardwareMap hardwareMap) {

    }

    @Override
    public void loop() {
        double rightY = -gamepad1.right_stick_y;
        servoPos += rightY * 0.01;

        }

    }

