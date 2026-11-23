package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ShooterV2 extends OpMode {

    // =====================================================
    // 1. HARDWARE
    // =====================================================

    private DcMotorEx motorShooter;
    private Servo servoAngulacao;
    private Servo servoTrava;

    // =====================================================
    // 2. CALIBRACAO DO MOTOR
    // =====================================================

    // Confirmar ticks por volta no eixo medido.
    double ticksPorVolta = 28.0;

    // Tolerancia para estabilizacao da velocidade.
    double toleranciaRPM = 50.0;

    // Tempo minimo dentro da tolerancia.
    double tempoEstavel = 0.25;

    // =====================================================
    // 3. PRESETS
    // Preencha com os valores calibrados.
    // =====================================================

    double rpmB = 2850.0;
    double angulacaoB = 200.0;
    double distanciaB = 110.0;

    double rpmX = 3000.0;
    double angulacaoX = 170.0;
    double distanciaX = 100.0;

    double rpmAlvo = 0.0;
    double anguloAlvo = 0.0;
    double distanciaAlvo = 0.0;

    String presetAtual = "Nenhum";

    // =====================================================
    // 4. CALIBRACAO DO SERVO E ENGRENAGENS
    // =====================================================

    double dentesServo = 19.0;
    double dentesCapuz = 16.0;

    double relacaoEngrenagem =
            dentesServo / dentesCapuz;

    // -1 ou +1 conforme o sentido real.
    double sentidoEngrenagem = -1.0;

    // Referencia horizontal do capuz.
    double anguloCapuzZero = 10.0;

    // Substituir pelo valor medido.
    double anguloServoZero = 100.0;

    // Limites calibrados do eixo.
    double anguloServoMinimo = 0.0;
    double anguloServoMaximo = 300.0;

    // Faixa de comando calibrada.
    double posicaoMinima = 0.15;
    double posicaoMaxima = 0.85;

    // =====================================================
    // 5. TRAVA
    // =====================================================

    // Ajustar conforme a montagem mecanica.
    double posicaoTravaFechada = 0.0;
    double posicaoTravaAberta = 0.5;

    boolean travaAberta = false;

    // =====================================================
    // 6. ESTADO DO SISTEMA
    // =====================================================

    double velocidadeRealRPM = 0.0;
    double erroRPM = 0.0;

    double anguloServoCalculado = 0.0;
    double posicaoServoCalculada = 0.5;

    boolean velocidadeAtingida = false;
    boolean anguloValido = false;

    long inicioEstabilidade = 0;

    // =====================================================
    // 7. INICIALIZACAO
    // =====================================================

    @Override
    public void init() {

        motorShooter = hardwareMap.get(
                DcMotorEx.class, "motorShooter"
        );

        servoAngulacao = hardwareMap.get(
                Servo.class, "servoAngulacao"
        );

        servoTrava = hardwareMap.get(
                Servo.class, "servoTrava"
        );

        motorShooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        motorShooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        motorShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        motorShooter.setPower(0.0);

        servoTrava.setPosition(posicaoTravaFechada);

        telemetry.addLine("Shooter inicializado");
        telemetry.addLine("Trava fechada");
        telemetry.update();
    }

    // =====================================================
    // 8. CONVERSOES DE VELOCIDADE
    // =====================================================

    private double rpmParaTicks(double rpm) {

        return rpm * ticksPorVolta / 60.0;
    }

    private double ticksParaRPM(double ticks) {

        return ticks * 60.0 / ticksPorVolta;
    }

    // =====================================================
    // 9. CALCULO DO SERVO
    // =====================================================
    private double calcularPosicaoServo(double anguloCapuz) {

        double variacaoCapuz =
                anguloCapuz - anguloCapuzZero;

        anguloServoCalculado =
                anguloServoZero
                        + sentidoEngrenagem
                        * variacaoCapuz
                        / relacaoEngrenagem;

        anguloValido =
                anguloServoCalculado >= anguloServoMinimo
                        && anguloServoCalculado <= anguloServoMaximo;

        if (!anguloValido) {
            return posicaoServoCalculada;
        }

        double proporcao =
                (anguloServoCalculado - anguloServoMinimo)
                        / (anguloServoMaximo - anguloServoMinimo);

        return Range.clip(
                posicaoMinima
                        + proporcao * (posicaoMaxima - posicaoMinima),
                posicaoMinima,
                posicaoMaxima
        );
    }

    // =====================================================
    // 10. SELECAO DOS PRESETS
    // =====================================================

    private void selecionarPresetB() {

        rpmAlvo = rpmB;
        anguloAlvo = angulacaoB;
        distanciaAlvo = distanciaB;

        presetAtual = "B";
    }

    private void selecionarPresetX() {

        rpmAlvo = rpmX;
        anguloAlvo = angulacaoX;
        distanciaAlvo = distanciaX;

        presetAtual = "X";
    }

    // =====================================================
    // 11. CONTROLE DO MOTOR
    // =====================================================

    private void controlarShooter() {

        if (rpmAlvo <= 0.0) {

            motorShooter.setVelocity(0.0);

            velocidadeRealRPM = 0.0;
            erroRPM = 0.0;
            velocidadeAtingida = false;

            inicioEstabilidade = 0;

            return;
        }

        motorShooter.setVelocity(
                rpmParaTicks(rpmAlvo)
        );

        double ticksReais =
                motorShooter.getVelocity();

        velocidadeRealRPM =
                ticksParaRPM(Math.abs(ticksReais));

        erroRPM =
                Math.abs(rpmAlvo - velocidadeRealRPM);

        if (erroRPM <= toleranciaRPM) {

            if (inicioEstabilidade == 0) {
                inicioEstabilidade = System.nanoTime();
            }

            double tempoAtual =
                    (System.nanoTime() - inicioEstabilidade)
                            / 1_000_000_000.0;

            velocidadeAtingida =
                    tempoAtual >= tempoEstavel;

        } else {

            inicioEstabilidade = 0;
            velocidadeAtingida = false;
        }
    }

    // =====================================================
    // 12. CONTROLE DO CAPUZ
    // =====================================================

    private void controlarAngulacao() {

        double novaPosicao =
                calcularPosicaoServo(anguloAlvo);

        if (anguloValido) {

            posicaoServoCalculada = novaPosicao;

            servoAngulacao.setPosition(
                    posicaoServoCalculada
            );
        }
    }

    // =====================================================
    // 13. CONTROLE DA TRAVA
    // =====================================================

    private void controlarTrava() {

        // A trava somente abre se o operador solicitar
        // e o motor estiver na velocidade estabilizada.

        if (gamepad1.right_bumper
                && velocidadeAtingida) {

            travaAberta = true;

        } else if (!gamepad1.right_bumper) {

            travaAberta = false;
        }

        if (travaAberta) {

            servoTrava.setPosition(
                    posicaoTravaAberta
            );

        } else {

            servoTrava.setPosition(
                    posicaoTravaFechada
            );
        }
    }

    // =====================================================
    // 14. LOOP PRINCIPAL
    // =====================================================

    @Override
    public void loop() {

        // Selecionar B.
        if (gamepad1.b) {
            selecionarPresetB();
        }

        // Selecionar X.
        if (gamepad1.x) {
            selecionarPresetX();
        }

        // Desligar shooter.
        if (gamepad1.a) {

            rpmAlvo = 0.0;
            presetAtual = "Desligado";

            travaAberta = false;
            servoTrava.setPosition(
                    posicaoTravaFechada
            );
        }

        controlarShooter();
        controlarAngulacao();
        controlarTrava();

        // =================================================
        // TELEMETRIA
        // =================================================

        telemetry.addData(
                "Preset", presetAtual
        );

        telemetry.addData(
                "RPM desejado", rpmAlvo
        );

        telemetry.addData(
                "RPM real", velocidadeRealRPM
        );

        telemetry.addData(
                "Erro RPM", erroRPM
        );

        telemetry.addData(
                "Velocidade estabilizada",
                velocidadeAtingida
        );

        telemetry.addData(
                "Angulo do capuz", anguloAlvo
        );

        telemetry.addData(
                "Angulo calculado do servo",
                anguloServoCalculado
        );

        telemetry.addData(
                "Posicao do servo",
                posicaoServoCalculada
        );

        telemetry.addData(
                "Angulo valido", anguloValido
        );

        telemetry.addData(
                "Trava aberta", travaAberta
        );

        telemetry.addData(
                "Distancia de referencia",
                distanciaAlvo
        );

        telemetry.update();
    }

    // =====================================================
    // 15. ENCERRAMENTO
    // =====================================================

    @Override
    public void stop() {

        motorShooter.setPower(0.0);

        servoTrava.setPosition(
                posicaoTravaFechada
        );
    }
}