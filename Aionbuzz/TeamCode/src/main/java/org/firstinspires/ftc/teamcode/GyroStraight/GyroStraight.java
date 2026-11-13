package org.firstinspires.ftc.teamcode.GyroStraight;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

@Autonomous(name = "AION - GyroStraight ")

public class GyroStraight extends LinearOpMode {

    // =========================================
    // 1. CONFIGURACOES AJUSTAVEIS
    // =========================================

    // Distancia do deslocamento.
    double distanciaTeste = 80.0; // cm
    // Diametro da roda convencional.
    double diametroRoda = 9.6; // cm
    // Encoder do Yellow Jacket 312 RPM.
    double ticksPorVolta = 537.6;
    // Velocidades reduzidas.
    double velocidadeMinima = 0.1;
    double velocidadeMaxima = 0.12;
    // Tempo de aceleracao gradual.
    double tempoAceleracao = 1.5;
    // Inicio da desaceleracao.
    double distanciaDesaceleracao = 55.0;
    // Tempo maximo do deslocamento.
    double tempoMaximoDeslocamento = 12.0;

    // =========================================
    // GYROSTRAIGHT - CONTROLE PD
    // =========================================

    // Ganho proporcional.
    double kpDirecao = -0.012;

    // Ganho derivativo.
    double kdDirecao = -0.012;

    // Limite absoluto da correcao.
    double limiteCorrecao = 0.6;

    // Tolerancia angular.
    double toleranciaAngular = 1.5;

    // =========================================
    // CONTROLE DE GIRO
    // =========================================

    double kpGiro = -0.026;
    double kdGiro = -0.026;

    double potenciaMinimaGiro = 0.1;
    double potenciaMaximaGiro = 0.15;

    double toleranciaGiro = 1.5;

    // Velocidade angular para considerar estabilizado.
    double velocidadeAngularFinal = 0.75;

    // Tempo maximo para cada giro.
    double tempoMaximoGiro = 7.0;

    // Numero de leituras estaveis necessarias.
    int leiturasNecessarias = 10;

    // =========================================
    // 2. HARDWARE
    // =========================================

    DcMotor motorDir;
    DcMotor motorEsq;

    IMU imu;

    ElapsedTime cronometro = new ElapsedTime();

    // =========================================
    // 3. VARIAVEIS
    // =========================================

    double circunferenciaRoda;

    double distanciaPercorrida;

    double anguloAtual;
    double erroAngular;

    double taxaAngular;
    double correcao;

    // Variaveis para angulo continuo.
    double ultimoYaw;
    double yawContinuo;

    // =========================================
    // 4. INICIALIZACAO
    // =========================================

    @Override
    public void runOpMode() {

        motorDir = hardwareMap.get(
                DcMotor.class, "motorDir");

        motorEsq = hardwareMap.get(
                DcMotor.class, "motorEsq");

        motorEsq.setDirection(
                DcMotor.Direction.REVERSE);

        motorDir.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        motorEsq.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        motorDir.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorEsq.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDir.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        motorEsq.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        // =====================================
        // IMU
        // =====================================

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientacao =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                );

        imu.initialize(
                new IMU.Parameters(orientacao));

        imu.resetYaw();

        // =====================================
        // CALCULOS
        // =====================================

        circunferenciaRoda =
                diametroRoda * Math.PI;

        telemetry.addLine("AION - GYROSTRAIGHT 180");

        telemetry.addData("Distancia", "%.1f cm",
                distanciaTeste);

        telemetry.addData("Diametro da roda", "%.1f cm",
                diametroRoda);

        telemetry.addData("Velocidade maxima", "%.2f",
                velocidadeMaxima);

        telemetry.addData("Massa", "5 kg");

        telemetry.addLine("Aguardando START");

        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            pararMotores();
            return;
        }

        // Inicializa o angulo continuo.
        ultimoYaw = lerYawBruto();
        yawContinuo = 0;

        // =====================================
        // ETAPA 1 - DESLOCAMENTO
        // =====================================

        boolean deslocamentoConcluido =
                gyroStraight(distanciaTeste);

        pararMotores();

        if (!deslocamentoConcluido || !opModeIsActive()) {
            return;
        }




        // =====================================
        // ETAPA 3 - RETORNO PARA 0 GRAUS
        // =====================================

        boolean retornoConcluido =
                girarPara(180.0);

        pararMotores();

        // =====================================
        // RESULTADO FINAL
        // =====================================

        telemetry.addLine("TESTE FINALIZADO");

        telemetry.addData("Deslocamento",
                deslocamentoConcluido);


        telemetry.addData("Retorno",
                retornoConcluido);

        telemetry.addData("Angulo final",
                "%.2f graus", yawContinuo);

        telemetry.update();
    }

    // =========================================
    // 5. LEITURA ANGULAR CONTINUA
    // =========================================

    double lerYawBruto() {

        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
    }

    void atualizarYawContinuo() {

        double yawAtual = lerYawBruto();

        double variacao =
                ((yawAtual - ultimoYaw + 540) % 360) - 180;

        yawContinuo += variacao;

        ultimoYaw = yawAtual;
    }

    // =========================================
    // 6. GYROSTRAIGHT
    // =========================================

    boolean gyroStraight(double distanciaAlvo) {

        motorDir.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorEsq.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDir.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        motorEsq.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        double referencia = yawContinuo;

        cronometro.reset();

        while (opModeIsActive()
                && cronometro.seconds()
                < tempoMaximoDeslocamento) {

            // =================================
            // ENCODERS
            // =================================

            double ticksDir =
                    Math.abs(motorDir.getCurrentPosition());

            double ticksEsq =
                    Math.abs(motorEsq.getCurrentPosition());

            double ticksMedios =
                    (ticksDir + ticksEsq) / 2.0;

            distanciaPercorrida =
                    (ticksMedios / ticksPorVolta)
                            * circunferenciaRoda;

            double restante =
                    distanciaAlvo - distanciaPercorrida;

            if (restante <= 0) {
                pararMotores();
                return true;
            }

            // =================================
            // IMU
            // =================================

            atualizarYawContinuo();

            anguloAtual = yawContinuo;

            erroAngular =
                    referencia - anguloAtual;

            AngularVelocity velocidade =
                    imu.getRobotAngularVelocity(
                            AngleUnit.DEGREES);

            taxaAngular =
                    velocidade.zRotationRate;

            // =================================
            // CONTROLE PD
            // =================================

            if (Math.abs(erroAngular)
                    <= toleranciaAngular) {

                correcao = 0;

            } else {

                correcao =
                        kpDirecao * erroAngular
                                - kdDirecao * taxaAngular;
            }

            // A correcao nao pode inverter
            // o sentido de deslocamento.

            // =================================
            // ACELERACAO
            // =================================

            double fatorAceleracao =
                    Range.clip(
                            cronometro.seconds()
                                    / tempoAceleracao,
                            0, 0.65
                    );

            double potenciaAceleracao =
                    velocidadeMinima
                            + (velocidadeMaxima
                            - velocidadeMinima)
                            * fatorAceleracao;

            // =================================
            // DESACELERACAO
            // =================================

            double fatorDesaceleracao =
                    Range.clip(
                            restante / distanciaDesaceleracao,
                            0, 0.65
                    );

            double potenciaDesaceleracao =
                    velocidadeMinima
                            + (velocidadeMaxima
                            - velocidadeMinima)
                            * fatorDesaceleracao;

            double potenciaBase =
                    Math.min(
                            potenciaAceleracao,
                            potenciaDesaceleracao
                    );

            // =================================
            // LIMITACAO DA CORRECAO
            // =================================

            double limiteAtual =
                    Math.min(
                            limiteCorrecao,
                            potenciaBase * 0.35
                    );

            correcao = Range.clip(
                    correcao,
                    -limiteAtual,
                    limiteAtual
            );

            // =================================
            // TANK DRIVE
            // =================================

            double potenciaEsq =
                    potenciaBase + correcao;

            double potenciaDir =
                    potenciaBase - correcao;

            motorDir.setPower(potenciaDir);
            motorEsq.setPower(potenciaEsq);

            // =================================
            // TELEMETRIA
            // =================================

            telemetry.addLine("GYROSTRAIGHT");

            telemetry.addData("Distancia",
                    "%.2f / %.2f cm",
                    distanciaPercorrida, distanciaAlvo);

            telemetry.addData("Angulo",
                    "%.2f", anguloAtual);

            telemetry.addData("Erro",
                    "%.2f", erroAngular);

            telemetry.addData("Taxa angular",
                    "%.2f", taxaAngular);

            telemetry.addData("Correcao",
                    "%.3f", correcao);

            telemetry.addData("Potencia direita",
                    "%.3f", potenciaDir);

            telemetry.addData("Potencia esquerda",
                    "%.3f", potenciaEsq);

            telemetry.update();

            idle();
        }

        pararMotores();

        return false;
    }

    // =========================================
    // 7. GIRO CONTROLADO
    // =========================================

    boolean girarPara(double alvo) {

        cronometro.reset();

        int leiturasEstaveis = 5;

        while (opModeIsActive()
                && cronometro.seconds()
                < tempoMaximoGiro) {

            atualizarYawContinuo();

            anguloAtual = yawContinuo;

            // Diferentemente do erro normalizado,
            // este erro usa o angulo acumulado.

            erroAngular =
                    alvo - anguloAtual;

            AngularVelocity velocidade =
                    imu.getRobotAngularVelocity(
                            AngleUnit.DEGREES);

            taxaAngular =
                    velocidade.zRotationRate;

            // =================================
            // VERIFICACAO DE ESTABILIDADE
            // =================================

            if (Math.abs(erroAngular)
                    <= toleranciaGiro
                    && Math.abs(taxaAngular)
                    <= velocidadeAngularFinal) {

                leiturasEstaveis++;

                pararMotores();

                if (leiturasEstaveis
                        >= leiturasNecessarias) {

                    return true;
                }

            } else {

                leiturasEstaveis = 0;

                // =================================
                // CONTROLE PD DO GIRO
                // =================================

                double comando =
                        kpGiro * erroAngular
                                - kdGiro * taxaAngular;

                comando = Range.clip(
                        comando,
                        -potenciaMaximaGiro,
                        potenciaMaximaGiro
                );

                // Evita comando excessivamente pequeno
                // quando ainda existe erro significativo.

                if (Math.abs(erroAngular) > toleranciaGiro
                        && Math.abs(comando)
                        < potenciaMinimaGiro
                        && comando * erroAngular > 0) {

                    comando =
                            Math.signum(erroAngular)
                                    * potenciaMinimaGiro;
                }

                motorDir.setPower(comando);
                motorEsq.setPower(-comando);
            }

            // =================================
            // TELEMETRIA
            // =================================

            telemetry.addLine("GIRO CONTROLADO");

            telemetry.addData("Alvo",
                    "%.2f", alvo);

            telemetry.addData("Angulo continuo",
                    "%.2f", anguloAtual);

            telemetry.addData("Erro",
                    "%.2f", erroAngular);

            telemetry.addData("Taxa angular",
                    "%.2f", taxaAngular);

            telemetry.addData("Comando",
                    "%.3f",
                    motorDir.getPower());

            telemetry.addData("Estabilidade",
                    "%d / %d",
                    leiturasEstaveis,
                    leiturasNecessarias);

            telemetry.update();

            idle();
        }
        pararMotores();
        return false;
    }
    // =========================================
    // 8. PARADA DOS MOTORES
    // =========================================
    void pararMotores() {

        motorDir.setPower(0);
        motorEsq.setPower(0);

        motorDir.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        motorEsq.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
    }
}