package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TelemetryTest extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Bem-vindos", "AION Robotics");
    }
    @Override
    public void init_loop(){
        float analogico = -gamepad1.left_stick_y;
        float analogicox = gamepad1.left_stick_x;
        if (analogico != 0.0) {
            telemetry.addData("Program", analogico);
            if (analogico > 0.0)
                telemetry.addData("Andando", "Frente");
            else if (analogico < 0.0) {
                telemetry.addData("Andando", "Trás");

            }
        }
        else
            telemetry.addData("Program", "Error");

        if (analogicox != 0.0) {
            telemetry.addData("Program", analogicox);
            if (analogicox > 0.0)
                telemetry.addData("Andando", "Frente");
            else if (analogicox < 0.0) {
                telemetry.addData("Andando", "Trás");

            }
        }
        else
            telemetry.addData("Program", "Error");

    }
    @Override
    public void loop() {

    }
}
