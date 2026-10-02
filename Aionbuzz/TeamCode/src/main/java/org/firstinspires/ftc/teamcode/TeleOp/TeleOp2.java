package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Teste de Arcade Drive")
public class TeleOp2 extends OpMode {
    private final double LIMITE = 0.5;

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;

    @Override
    public void init() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDireito");

        motorEsquerdo.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDireito.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double LeftY = -gamepad1.left_stick_y;
        double Spin = gamepad1.right_stick_x;

        double esquerda = Math.max(-LIMITE, Math.min(LIMITE, LeftY + Spin));
        double direita  = Math.max(-LIMITE, Math.min(LIMITE, LeftY - Spin));

        motorEsquerdo.setPower(esquerda);
        motorDireito.setPower(direita);

        telemetry.addData("Esquerda", "%.2f", esquerda);
        telemetry.addData("Direita", "%.2f", direita);
        telemetry.addData("Encoder", motorEsquerdo.getCurrentPosition());
    }

    @Override
    public void stop() {
            motorEsquerdo.setPower(0);
            motorDireito.setPower(0);
    }
}

