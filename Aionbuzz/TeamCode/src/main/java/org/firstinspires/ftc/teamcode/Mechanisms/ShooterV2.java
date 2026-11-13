package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ShooterV2 extends OpMode {
    DcMotorEx motorShooter = null;
    private Servo servoGate;
    private double servoBloq = 0.0;
    private double servoLib = 1.0;
    private boolean bloqueado = true;
    final static double maxRPM = 6000;
    double limite = 0.0;
    double ticksPorVolta = 28.0; // ticks emitido pelo motor modificado
    double tolerancia = 0.05; // diferença entre a velocidade solicitada e atual
    long temoEstavelNanos = 250_000_000L;
    long inicioEstabilidade = -1;
    boolean pronto = false;

    @Override
    public void init() {
        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setPower(0);

        servoGate = hardwareMap.get(Servo.class, "servoGate");
        bloquear(); // servo que iremos usar para o mecanismo dde trava
    }

    public void bloquear() {
        servoGate.setPosition(servoBloq);
    }

    public void liberar() {
        servoGate.setPosition(servoLib);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            limite = 0.15;
        } else if (gamepad1.b) {
            limite = 0.25;
        } else if (gamepad1.y) {
            limite = 0.3;
        } else if (gamepad1.x) {
            limite = 0.35;}
        else if (gamepad1.right_bumper){
            limite = limite*-1;}
        else {
            limite = 0;
        }

        double alvoRPM = maxRPM * limite;
        double alvoTicks = (alvoRPM * ticksPorVolta) / 60.0;

        motorShooter.setVelocity(-alvoTicks);
        double velocidadeTicks = Math.abs(motorShooter.getVelocity());
        double velocidadeRPM = velocidadeTicks * 60 / ticksPorVolta;

        boolean dentroDaFaixa =
                alvoRPM > 00
                        && velocidadeRPM >=
                        alvoRPM * (1.0 - tolerancia)
                        && velocidadeRPM <=
                        alvoRPM * (1.0 + tolerancia);

        long agora = System.nanoTime();
        if (dentroDaFaixa) {

            if (inicioEstabilidade < 0) {
                inicioEstabilidade = agora;
            }
            pronto = (agora - inicioEstabilidade)
                    >= temoEstavelNanos;
        } else {
            inicioEstabilidade = -1;
            pronto = false;
        }

        telemetry.addData("Alvo RPM", "%.0f", alvoRPM);
        telemetry.addData("RPM real", "%.0f", velocidadeRPM);
        telemetry.addData("Shooter", pronto ? "PRONTO PARA LANÇAR" : "AGUARDANDO VELOCIDADE");
        telemetry.update();

        // Limitador do shooter
        if (pronto) {
            liberar();
        } else {
            bloquear();
        }
    }

    @Override
    public void stop() {
        if (motorShooter != null) {
            motorShooter.setPower(0);
        }
    }
}