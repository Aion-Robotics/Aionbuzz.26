package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Shooter", group = "Testes")
public class TestShooter extends OpMode {

    // ==================== CONSTANTES ====================
    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_POR_VOLTA = 28.0;
    private static final double TOLERANCIA = 0.05;
    private static final long TEMPO_ESTAVEL_NANOS = 250_000_000L; // 0.25 segundos

    private static final double SERVO_BLOQ = 0.0;
    private static final double SERVO_LIB = 1.0;

    // ==================== HARDWARE ====================
    private DcMotorEx motorShooter;
    private Servo servoGate;

    // ==================== VARIÁVEIS DE ESTADO ====================
    private double limite = 0.0;
    private long inicioEstabilidade = -1;
    private boolean pronto = false;

    @Override
    public void init() {
        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setPower(0);

        servoGate = hardwareMap.get(Servo.class, "servoGate");
        bloquear();

        telemetry.addData("Status", "Inicializado com sucesso");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Seleção de limite de velocidade pelo Gamepad 1
        if (gamepad1.a) {
            limite = 0.80;
        } else if (gamepad1.b) {
            limite = 0.85;
        } else if (gamepad1.y) {
            limite = 0.90;
        } else if (gamepad1.x) {
            limite = 0.95;
        } else if (gamepad1.right_trigger > 0.5) { // Corrigido para verificar o float do gatilho
            limite = 0.0;
        }

        // Cálculos de Alvo e Velocidade
        double alvoRPM = MAX_RPM * limite;
        double alvoTicks = (alvoRPM * TICKS_POR_VOLTA) / 60.0;

        motorShooter.setVelocity(alvoTicks);

        double velocidadeTicks = Math.abs(motorShooter.getVelocity());
        double velocidadeRPM = (velocidadeTicks * 60.0) / TICKS_POR_VOLTA;

        // Verificação se o shooter está dentro da faixa de velocidade tolerada
        boolean dentroDaFaixa = (alvoRPM > 0) &&
                (velocidadeRPM >= alvoRPM * (1.0 - TOLERANCIA)) &&
                (velocidadeRPM <= alvoRPM * (1.0 + TOLERANCIA));

        long agora = System.nanoTime();
        if (dentroDaFaixa) {
            if (inicioEstabilidade < 0) {
                inicioEstabilidade = agora;
            }
            pronto = (agora - inicioEstabilidade) >= TEMPO_ESTAVEL_NANOS;
        } else {
            inicioEstabilidade = -1;
            pronto = false;
        }

        // Controle do Servo (Gate) com base na estabilidade do shooter
        if (pronto) {
            liberar();
        } else {
            bloquear();
        }

        // Telemetria
        telemetry.addData("Alvo RPM", "%.0f", alvoRPM);
        telemetry.addData("RPM Real", "%.0f", velocidadeRPM);
        telemetry.addData("Status Shooter", pronto ? "PRONTO PARA LANÇAR" : "AGUARDANDO VELOCIDADE");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motorShooter != null) {
            motorShooter.setPower(0);
        }
    }

    // ==================== MÉTODOS AUXILIARES ====================
    private void bloquear() {
        servoGate.setPosition(SERVO_BLOQ);
    }

    private void liberar() {
        servoGate.setPosition(SERVO_LIB);
    }
}