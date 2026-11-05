package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
@Autonomous

public class GyronStraight extends LinearOpMode {


        // =========================================
        // 1. CONFIGURACOES AJUSTAVEIS
        // =========================================

        // Distancia do deslocamento.
        double distanciaTeste = 80.0; // cm

        // Roda de 96 mm.
        double diametroRoda = 9.6; // cm

        // Encoder do Yellow Jacket.
        // Confirmar se corresponde a uma volta da roda.
        double ticksPorVolta = 537.6;

        // Velocidades do deslocamento.
        double velocidadeMinima = 0.10;
        double velocidadeMaxima = 0.22;

        // Tempo para acelerar.
        double tempoAceleracao = 1.5;

        // Distancia para iniciar desaceleracao.
        double distanciaDesaceleracao = 25.0;

        // =========================================
        // GYROSTRAIGHT - CONTROLE PD
        // =========================================

        // Ganho proporcional.
        double kpDirecao = 12;

        // Ganho derivativo.
        double kdDirecao = 0.5;

        // Limite da correcao.
        double limiteCorrecao = 0.10;

        // Tolerancia angular.
        double toleranciaAngular = 1.5;

        // =========================================
        // CONTROLE DO GIRO
        // =========================================

        double kpGiro = 0.012;
        double kdGiro = 0.02;

        double limiteGiro = 0.18;

        // Tolerancia do giro.
        double toleranciaGiro = 1.5;

        // Velocidade angular maxima para considerar parado.
        double velocidadeAngularFinal = 5.0;

        // Tempo maximo de cada movimento.
        double tempoMaximoDeslocamento = 2.0;
        double tempoMaximoGiro = 4.0;

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

        double anguloInicial;
        double anguloAtual;

        double erroAngular;
        double taxaAngular;

        double correcao;

        // =========================================
        // 4. INICIALIZACAO
        // =========================================

        @Override
        public void runOpMode() {

            motorDir = hardwareMap.get(
                    DcMotor.class, "motorDir");

            motorEsq = hardwareMap.get(
                    DcMotor.class, "motorEsq");

            // Mesmo padrao do RepositoryRobot.
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

            telemetry.addLine("AION - GYROSTRAIGHT");

            telemetry.addData("Roda", "%.2f cm",
                    diametroRoda);

            telemetry.addData("Circunferencia", "%.2f cm",
                    circunferenciaRoda);

            telemetry.addData("Massa", "5 kg");

            telemetry.addLine("Aguardando START");

            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                pararMotores();
                return;
            }

            anguloInicial = lerAngulo();

            // =====================================
            // ETAPA 1 - GYROSTRAIGHT
            // =====================================

            boolean deslocamentoConcluido =
                    gyroStraight(distanciaTeste, anguloInicial);

            pararMotores();

            if (!deslocamentoConcluido || !opModeIsActive()) {
                return;
            }

            sleep(500);

            // =====================================
            // ETAPA 2 - GIRO DE 90 GRAUS
            // =====================================

            double anguloAlvo =
                    anguloInicial + 180.0;

            boolean giroConcluido =
                    girarPara(anguloAlvo);

            pararMotores();

            if (!giroConcluido || !opModeIsActive()) {
                return;
            }

            sleep(500);

            // =====================================
            // ETAPA 3 - RETORNO ANGULAR
            // =====================================

            boolean retornoConcluido =
                    girarPara(anguloInicial);

            pararMotores();

            telemetry.addLine("TESTE FINALIZADO");

            telemetry.addData("Deslocamento",
                    deslocamentoConcluido);

            telemetry.addData("Giro",
                    giroConcluido);

            telemetry.addData("Retorno",
                    retornoConcluido);

            telemetry.addData("Angulo final",
                    "%.2f graus", lerAngulo());

            telemetry.update();
        }

        // =========================================
        // 5. GYROSTRAIGHT
        // =========================================

        boolean gyroStraight(double distanciaAlvo,
                             double referencia) {

            motorDir.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorEsq.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorDir.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);

            motorEsq.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);

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
                // LEITURA DO IMU
                // =================================

                anguloAtual = lerAngulo();

                erroAngular =
                        calcularErro(referencia, anguloAtual);

                AngularVelocity velocidade =
                        imu.getRobotAngularVelocity(
                                AngleUnit.DEGREES);

                taxaAngular =
                        velocidade.zRotationRate;

                // =================================
                // CONTROLE PD
                // =================================

                // O termo derivativo utiliza a velocidade
                // angular para amortecer a correcao.

                if (Math.abs(erroAngular)
                        <= toleranciaAngular) {

                    correcao = 0;

                } else {

                    correcao =
                            kpDirecao * erroAngular
                                    - kdDirecao * taxaAngular;
                }

                correcao = Range.clip(
                        correcao,
                        -limiteCorrecao,
                        limiteCorrecao
                );

                // =================================
                // ACELERACAO
                // =================================

                double fatorAceleracao =
                        Range.clip(
                                cronometro.seconds()
                                        / tempoAceleracao,
                                0, 1
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
                                0, 1
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
                // TANK DRIVE
                // =================================

                double potenciaDir =
                        potenciaBase + correcao;

                double potenciaEsq =
                        potenciaBase - correcao;

                motorDir.setPower(
                        Range.clip(potenciaDir, -0.30, 0.30));

                motorEsq.setPower(
                        Range.clip(potenciaEsq, -0.30, 0.30));

                // =================================
                // TELEMETRIA
                // =================================

                telemetry.addLine("GYROSTRAIGHT");

                telemetry.addData("Distancia",
                        "%.2f / %.2f cm",
                        distanciaPercorrida, distanciaAlvo);

                telemetry.addData("Angulo inicial",
                        "%.2f", referencia);

                telemetry.addData("Angulo atual",
                        "%.2f", anguloAtual);

                telemetry.addData("Erro angular",
                        "%.2f", erroAngular);

                telemetry.addData("Taxa angular",
                        "%.2f", taxaAngular);

                telemetry.addData("Correcao",
                        "%.3f", correcao);

                telemetry.addData("Potencia base",
                        "%.3f", potenciaBase);

                telemetry.addData("Motor direito",
                        "%.3f", potenciaDir);

                telemetry.addData("Motor esquerdo",
                        "%.3f", potenciaEsq);

                telemetry.update();

                idle();
            }

            pararMotores();

            return false;
        }

        // =========================================
        // 6. GIRO PELO IMU
        // =========================================

        boolean girarPara(double alvo) {

            cronometro.reset();

            int leiturasEstaveis = 0;

            while (opModeIsActive()
                    && cronometro.seconds() < tempoMaximoGiro) {

                anguloAtual = lerAngulo();

                erroAngular =
                        calcularErro(alvo, anguloAtual);

                AngularVelocity velocidade =
                        imu.getRobotAngularVelocity(
                                AngleUnit.DEGREES);

                taxaAngular =
                        velocidade.zRotationRate;

                if (Math.abs(erroAngular) <= toleranciaGiro
                        && Math.abs(taxaAngular)
                        <= velocidadeAngularFinal) {

                    leiturasEstaveis++;

                    pararMotores();

                    if (leiturasEstaveis >= 5) {
                        return true;
                    }

                } else {

                    leiturasEstaveis = 0;

                    double comando =
                            kpGiro * erroAngular
                                    - kdGiro * taxaAngular;

                    comando = Range.clip(
                            comando,
                            -limiteGiro,
                            limiteGiro
                    );

                    motorDir.setPower(comando);
                    motorEsq.setPower(-comando);
                }

                telemetry.addLine("GIRO PELO IMU");

                telemetry.addData("Alvo",
                        "%.2f", alvo);

                telemetry.addData("Atual",
                        "%.2f", anguloAtual);

                telemetry.addData("Erro",
                        "%.2f", erroAngular);

                telemetry.addData("Taxa angular",
                        "%.2f", taxaAngular);

                telemetry.addData("Leituras estaveis",
                        leiturasEstaveis);

                telemetry.update();

                idle();
            }

            pararMotores();

            return false;
        }

        // =========================================
        // 7. LEITURA DO ANGULO
        // =========================================

        double lerAngulo() {

            return imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);
        }

        // =========================================
        // 8. CALCULO DO ERRO ANGULAR
        // =========================================

        double calcularErro(double alvo, double atual) {

            return ((alvo - atual + 540) % 360) - 180;
        }

        // =========================================
        // 9. PARADA
        // =========================================

        void pararMotores() {

            motorDir.setPower(0);
            motorEsq.setPower(0);
        }
    }