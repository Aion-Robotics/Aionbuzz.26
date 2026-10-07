package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Teleop extends OpMode {

    double LeftY = -gamepad1.left_stick_y;
    double RightY = -gamepad1.right_stick_y;
    DcMotor esq, dir;


    @Override
    public void init() {
        esq = hardwareMap.get(DcMotor.class, "motorEsq");
        dir = hardwareMap.get(DcMotor.class, "motorDir");
        dir.setDirection(DcMotorSimple.Direction.REVERSE);
        esq.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        esq.setPower(LeftY);
        dir.setPower(RightY);

    }
}
