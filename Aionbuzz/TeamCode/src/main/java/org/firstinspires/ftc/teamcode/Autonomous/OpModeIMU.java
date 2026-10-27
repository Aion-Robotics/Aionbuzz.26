package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class OpModeIMU extends OpMode {

    /*
    DECLARAÇÃO DOS COMPONENTES
    */

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;
    private IMU imu;

    private final ElapsedTime tempo = new ElapsedTime();
    // cronometro utilizado para contagem de tempo
    private int etapa = 0;
    //identifica a etapa a ser executada


    /*
    CONFIGURAÇÕES DE MOVIMENTO
     */

    private static final double POTENCIA_FRENTE = 0.50;
    private static final double POTENCIA_MINIMA_GIRO = 0.18;
    private static final double POTENCIA_MAXIMA_GIRO = 0.60;

    private static final double ANGULO_GIRO = 90.0;
    private static final double TOLERANCIA_GRAUS = 2.0;

    /*
    TEMPO DOS DESLOCAMENTOS
     */

    private static final double TEMPO_PRIMEIRO_LADO = 3.50;
    private static final double TEMPO_SEGUNDO_LADO = 2.60;
    private static final double TEMPO_TERCEIRO_LADO = 2.55;
    private static final double TEMPO_QUARTO_LADO = 2.50;

    private static final double TEMPO_PAUSA = 0.20;


    /*
    VARIÁVEIS DA IMU
     */

    private double alvoHeading = 0.0;
    // angulo que o robo deve atingit durante o próximo giro


    @Override
    public void init() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDir");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationHub =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        /*
         * Configuração da orientação física do Expansion Hub.
         *
         * Esta configuração considera:
         *
         * - Logo do Hub voltado para cima;
         * - Porta USB voltada para a trás do robô.
         *
         * A orientação deve corresponder à montagem real do Hub.
         */

        imu.initialize(new IMU.Parameters(orientationHub));
        // inicializa a IMU utilizando a orientação informada

        motorDireito.setDirection(DcMotorSimple.Direction.REVERSE);
        // inverte a orientação do motor
        motorDireito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // faz os motores frearem em potência zero

        parar();

        telemetry.addLine("Inicializado");
        telemetry.addLine("IMU configurada");
        telemetry.update();
    }

    // INICIO DO AUTÔNOMO
    @Override
    public void start(){
        imu.resetYaw();
        /*
         * Define a orientação atual do robô como 0°.
         *
         * A partir deste momento:
         *
         * - 0° representa a direção inicial;
         * - 90° representa um giro de 90°;
         * - 180° representa meia-volta;
         * - -90° representa um giro no sentido contrário.
         */

        mudarEtapa(1);
    }

    @Override
    public void loop() {
        switch (etapa){

        case 1:
            andarFrente();

            if (tempo.seconds() >= TEMPO_PRIMEIRO_LADO) {
                parar();
                mudarEtapa(2);
            }

            break;

        case 2:
            parar();

            if (tempo.seconds() >= TEMPO_PAUSA){

            iniciarGiroEsquerda(3);

            }
            break;

            case 3:

                if (girarAteAlvo()) {
                    parar();
                    mudarEtapa(4);
                }
                break;

            case 4:
                parar();

                if (tempo.seconds() >= TEMPO_PAUSA) {
                    mudarEtapa(5);
                }
                break;

            case 5:
                andarFrente();

                if (tempo.seconds() >= TEMPO_SEGUNDO_LADO) {
                    parar();
                    mudarEtapa(6);
                }
                break;

            case 6:
                parar();

                if (tempo.seconds() >= TEMPO_PAUSA) {
                    iniciarGiroEsquerda(7);
                }
                break;

            case 7:
                if (girarAteAlvo()) {
                    parar();
                    mudarEtapa(8);
                }
                break;

            case 8:
                parar();

                if (tempo.seconds() >= TEMPO_PAUSA) {
                    mudarEtapa(9);
                }
                break;

            case 9:
                andarFrente();

                if (tempo.seconds() >= TEMPO_TERCEIRO_LADO){
                    parar();
                    iniciarGiroEsquerda(10);
                }
                break;

            case 10:
                if (girarAteAlvo()) {
                    parar();
                    mudarEtapa(11);
                }
                break;

            case 11:
                parar();

                if (tempo.seconds() >= TEMPO_PAUSA) {
                    mudarEtapa(12);
                }
                break;

            case 12:
                andarFrente();

                if (tempo.seconds() >= TEMPO_QUARTO_LADO) {
                    parar();
                    mudarEtapa(13);
                }
                break;

            case 13:
                parar();
                break;

            default:
                parar();
                break;
        }
        atualizarTelemetry();

    }

    @Override
    public void stop() {
        parar(); // desliga os dois motores no fim da etapa
    }


    // MOVIMENTOS BÁSICOS
    private void andarFrente() {
        motorEsquerdo.setPower(POTENCIA_FRENTE);
        motorDireito.setPower(POTENCIA_FRENTE);
    }

    private void parar(){
        motorEsquerdo.setPower(0.0);
        motorDireito.setPower(0.0);
        // zera a potência dos dois motores
    }

    // CONTROLE DAS ETAPAS
    private void mudarEtapa(int novaEtapa) {
        etapa = novaEtapa;
        // atualiza etapa atual
        tempo.reset();
        // zera o cronometro para iniciar nova etapa apartir do zero
    }

    // CONTROLE DE GIRO COM IMU
    private void iniciarGiroEsquerda(int etapaDoGiro) {
        alvoHeading = normalizarAngulo(obterHeading() + ANGULO_GIRO);
        /* Define o alvo como 90° à esquerda da orientação atual.
         *
         * Nesta configuração, considera-se que o valor do yaw
         * aumenta quando o robô gira para a esquerda.
         */

        mudarEtapa(etapaDoGiro);
    }

    private boolean girarAteAlvo() {
        double erro = obterErroHeading();
        // Calcula diferença entre o alvo e a orientação atual

        if (Math.abs(erro) <= TOLERANCIA_GRAUS) {
            parar();
            return true;
        }

        /*
         * Se o erro estiver dentro da tolerância,
         * o giro foi concluído.
         */

        double potencia = Math.abs(erro) * 0.01;
        /*
         * Controle proporcional simples:
         *
         * - Erro grande: utiliza mais potência;
         * - Erro pequeno: utiliza menos potência.
         */

        potencia = Math.min(potencia, POTENCIA_MAXIMA_GIRO);
        // Limita a potência máxima do giro.
        potencia = Math.max(potencia, POTENCIA_MINIMA_GIRO);
        // Evita uma potência baixa demais para vencer o atrito.

        if (erro > 0.0) {
            motorEsquerdo.setPower(-potencia);
            motorDireito.setPower(potencia);
        } else {
            motorDireito.setPower(-potencia);
            motorEsquerdo.setPower(potencia);
        }

        return false;
    }

    // LEITURA DOS ÂNGULOS IMU

    private double obterHeading() {

        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
          //Retorna o yaw atual do robô em graus.

    }

    private double obterErroHeading() {
        double erro = alvoHeading - obterHeading();
        //Calcula diferença entre ângulo desejado e ângulo atual

        return normalizarAngulo(erro);
        /*
         * Normaliza o erro para sempre utilizar o menor
         * caminho entre -180° e +180°.
         */
    }

    private double normalizarAngulo(double angulo) {
        while (angulo > 180.0) {
            angulo -= 360.0;
        }
        while (angulo <= -180.0) {
            angulo += 360.0;
        }
        return angulo;

        /*
         * Mantém o ângulo dentro do intervalo:
         *
         * -180° < ângulo <= 180°
         */
    }

    // TELEMETRIA

    private void atualizarTelemetry() {
        telemetry.addData("Etapa", etapa);
        telemetry.addData("Tempo da Etapa", "%.2f s", tempo.seconds());
        telemetry.addData("Heading Atual", "%.2f graus", alvoHeading);
        telemetry.addData("Erro do Giro", "%.2f graus", obterErroHeading());
        telemetry.addData("Motor Esquerdo", "%.2f", motorEsquerdo.getPower());
        telemetry.addData("Motor Direito", "%.2f", motorDireito.getPower());
        telemetry.update();
    }
}
