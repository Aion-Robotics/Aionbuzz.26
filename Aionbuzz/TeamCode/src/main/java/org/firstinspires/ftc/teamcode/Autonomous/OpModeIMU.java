package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Autonomo IMU", group = "Autonomous")
public class OpModeIMU extends OpMode {

    // =========================
    // Hardware
    // =========================

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;
    private IMU imu;

    // =========================
    // Maquina de estados
    // =========================

    private enum Estado {
        LADO_1,
        PAUSA_ANTES_GIRO_1,
        GIRO_1,
        PAUSA_DEPOIS_GIRO_1,

        LADO_2,
        PAUSA_ANTES_GIRO_2,
        GIRO_2,
        PAUSA_DEPOIS_GIRO_2,

        LADO_3,
        PAUSA_ANTES_GIRO_3,
        GIRO_3,
        PAUSA_DEPOIS_GIRO_3,

        LADO_4,
        FINALIZADO,
        FALHA
    }

    private Estado estadoAtual = Estado.FINALIZADO;
    private final ElapsedTime tempoEstado = new ElapsedTime();
    private final ElapsedTime tempoAutonomo = new ElapsedTime();

    // =========================
    // Configuraçõ
    //
    // es de movimento
    // =========================

    private static final double POTENCIA_FRENTE = 0.50;

    private static final double ANGULO_GIRO = 90.0;
    private static final double TOLERANCIA_GRAUS = 3.5;

    // Controle proporcional do giro
    private static final double KP_GIRO = 0.005;
    private static final double POTENCIA_MINIMA_GIRO = 0.10;
    private static final double POTENCIA_MAXIMA_GIRO = 0.55;

    // A potencia minima so e aplicada longe do alvo.
    private static final double ERRO_MINIMO_PARA_POTENCIA_MINIMA = 10.0;

    // Se o giro durar mais que isso, o robo para por seguranca.
    private static final double TEMPO_MAXIMO_GIRO = 3.0;

    // =========================
    // Tempos dos deslocamentos
    // =========================

    private static final double TEMPO_LADO_1 = 3.25;
    private static final double TEMPO_LADO_2 = 2.60;
    private static final double TEMPO_LADO_3 = 2.55;
    private static final double TEMPO_LADO_4 = 2.50;
    private static final double TEMPO_PAUSA = 0.20;

    // =========================
    // Variaveis da IMU
    // =========================

    private double alvoHeading = 0.0;

    /**
     * Usar +1 se o heading aumentar ao girar para a esquerda.
     * Usar -1 se o heading diminuir ao girar para a esquerda.
     */
    private static final double SINAL_GIRO_ESQUERDA = 1.0;

    @Override
    public void init() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDir");
        imu = hardwareMap.get(IMU.class, "imu");

        // Esta orientacao precisa corresponder a montagem real do Hub.
        RevHubOrientationOnRobot orientationHub =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientationHub));

        // Com um motor invertido, os dois motores devem mover o robo para frente
        // quando recebem a mesma potencia positiva.
        motorEsquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDireito.setDirection(DcMotorSimple.Direction.FORWARD);

        motorEsquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorEsquerdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDireito.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        parar();

        telemetry.addLine("Inicializado");
        telemetry.addLine("IMU configurada");
        telemetry.addLine("Verifique a orientacao fisica do Hub");
        telemetry.update();
    }

    @Override
    public void start() {
        imu.resetYaw();
        tempoAutonomo.reset();
        mudarEstado(Estado.LADO_1);
    }

    @Override
    public void loop() {
        switch (estadoAtual) {
            case LADO_1:
                executarDeslocamento(TEMPO_LADO_1, Estado.PAUSA_ANTES_GIRO_1);
                break;

            case PAUSA_ANTES_GIRO_1:
                executarPausa(Estado.GIRO_1);
                break;

            case GIRO_1:
                executarGiro(Estado.PAUSA_DEPOIS_GIRO_1);
                break;

            case PAUSA_DEPOIS_GIRO_1:
                executarPausa(Estado.LADO_2);
                break;

            case LADO_2:
                executarDeslocamento(TEMPO_LADO_2, Estado.PAUSA_ANTES_GIRO_2);
                break;

            case PAUSA_ANTES_GIRO_2:
                executarPausa(Estado.GIRO_2);
                break;

            case GIRO_2:
                executarGiro(Estado.PAUSA_DEPOIS_GIRO_2);
                break;

            case PAUSA_DEPOIS_GIRO_2:
                executarPausa(Estado.LADO_3);
                break;

            case LADO_3:
                executarDeslocamento(TEMPO_LADO_3, Estado.PAUSA_ANTES_GIRO_3);
                break;

            case PAUSA_ANTES_GIRO_3:
                executarPausa(Estado.GIRO_3);
                break;

            case GIRO_3:
                executarGiro(Estado.PAUSA_DEPOIS_GIRO_3);
                break;

            case PAUSA_DEPOIS_GIRO_3:
                executarPausa(Estado.LADO_4);
                break;

            case LADO_4:
                executarDeslocamento(TEMPO_LADO_4, Estado.FINALIZADO);
                break;

            case FINALIZADO:
                parar();
                break;

            case FALHA:
                parar();
                break;

            default:
                parar();
                mudarEstado(Estado.FALHA);
                break;
        }

        atualizarTelemetry();
    }

    @Override
    public void stop() {
        parar();
    }

    // =========================
    // Execução dos estados
    // =========================

    private void executarDeslocamento(double duracao, Estado proximoEstado) {
        andarFrente();

        if (tempoEstado.seconds() >= duracao) {
            parar();
            mudarEstado(proximoEstado);
        }
    }

    private void executarPausa(Estado proximoEstado) {
        parar();

        if (tempoEstado.seconds() >= TEMPO_PAUSA) {
            if (proximoEstado == Estado.GIRO_1
                    || proximoEstado == Estado.GIRO_2
                    || proximoEstado == Estado.GIRO_3) {
                iniciarGiroEsquerda(proximoEstado);
            } else {
                mudarEstado(proximoEstado);
            }
        }
    }

    private void executarGiro(Estado proximoEstado) {
        if (tempoEstado.seconds() >= TEMPO_MAXIMO_GIRO) {
            parar();
            telemetry.addLine("FALHA: tempo maximo do giro excedido");
            mudarEstado(Estado.FALHA);
            return;
        }

        if (girarAteAlvo()) {
            parar();
            mudarEstado(proximoEstado);
        }
    }

    // =========================
    // Movimentos basicos
    // =========================

    private void andarFrente() {
        motorEsquerdo.setPower(POTENCIA_FRENTE);
        motorDireito.setPower(POTENCIA_FRENTE);
    }

    private void parar() {
        motorEsquerdo.setPower(0.0);
        motorDireito.setPower(0.0);
    }

    // =========================
    // Controle da maquina de estados
    // =========================

    private void mudarEstado(Estado novoEstado) {
        estadoAtual = novoEstado;
        tempoEstado.reset();
    }

    // =========================
    // Controle do giro com IMU
    // =========================

    private void iniciarGiroEsquerda(Estado estadoDoGiro) {
        alvoHeading = normalizarAngulo(
                obterHeading() + (SINAL_GIRO_ESQUERDA * ANGULO_GIRO));

        mudarEstado(estadoDoGiro);
    }

    private boolean girarAteAlvo() {
        double erro = obterErroHeading();
        double erroAbsoluto = Math.abs(erro);

        if (erroAbsoluto <= TOLERANCIA_GRAUS) {
            parar();
            return true;
        }

        double potencia = erroAbsoluto * KP_GIRO;
        potencia = Math.min(potencia, POTENCIA_MAXIMA_GIRO);

        // Evita que o robo fique sem forca longe do alvo,
        // mas permite uma aproximacao suave perto do alvo.
        if (erroAbsoluto > ERRO_MINIMO_PARA_POTENCIA_MINIMA) {
            potencia = Math.max(potencia, POTENCIA_MINIMA_GIRO);
        }

        // O erro positivo representa a direcao necessaria para atingir o alvo.
        // Estes sinais devem ser confirmados no teste fisico.
        if (erro > 0.0) {
            motorEsquerdo.setPower(-potencia);
            motorDireito.setPower(potencia);
        } else {
            motorEsquerdo.setPower(potencia);
            motorDireito.setPower(-potencia);
        }

        return false;
    }

    // =========================
    // Leitura e normalizacao do heading
    // =========================

    private double obterHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double obterErroHeading() {
        double erro = alvoHeading - obterHeading();
        return normalizarAngulo(erro);
    }

    private double normalizarAngulo(double angulo) {
        while (angulo > 180.0) {
            angulo -= 360.0;
        }

        while (angulo <= -180.0) {
            angulo += 360.0;
        }

        return angulo;
    }

    // =========================
    // Telemetria
    // =========================

    private void atualizarTelemetry() {
        telemetry.addData("Estado", estadoAtual);
        telemetry.addData("Tempo do estado", "%.2f s", tempoEstado.seconds());
        telemetry.addData("Tempo total", "%.2f s", tempoAutonomo.seconds());
        telemetry.addData("Heading atual", "%.2f graus", obterHeading());
        telemetry.addData("Heading alvo", "%.2f graus", alvoHeading);
        telemetry.addData("Erro do giro", "%.2f graus", obterErroHeading());
        telemetry.addData("Motor esquerdo", "%.2f", motorEsquerdo.getPower());
        telemetry.addData("Motor direito", "%.2f", motorDireito.getPower());
        telemetry.update();
    }
}