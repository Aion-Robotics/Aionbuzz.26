package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class testeControleAngular extends LinearOpMode {

    // =========================================
    // CONFIGURACOES AJUSTAVEIS
    // =========================================

    // Distancia que o robo deve percorrer.
    double distanciaTeste = 100.0; // centimetros

    // Diametro da roda.
    double diametroRoda = 9.6; // centimetros

    // Encoder do Yellow Jacket 312 RPM.
    double ticksPorVolta = 537.6;

    // Velocidade inicial do deslocamento.
    double velocidadeBase = 0.10;

    // Controle de direcao durante o deslocamento.
    double kpDirecao = 1.2;

    // Controle proporcional do giro.
    double kpGiro = 1.2;

    // Potencia minima e maxima durante o giro.
    double potenciaMinimaGiro = 0.12;
    double potenciaMaximaGiro = 0.30;

    // Tolerancia para considerar o giro concluido.
    double toleranciaAngular = 1.5;

    // Tempo maximo permitido para cada giro.
    double tempoMaximoGiro = 5.0;

    // =========================================
    // HARDWARE
    // =========================================

    DcMotor motorDir;
    DcMotor motorEsq;
    IMU imu;

    // =========================================
    // VARIAVEIS
    // =========================================

    double anguloInicial;
    double anguloAtual;

    double distanciaPercorrida;
    double erroAngular;

    double circunferenciaRoda;
    double ticksAlvo;

    ElapsedTime cronometro = new ElapsedTime();

    @Override
    public void runOpMode() {

        // =========================================
        // INICIALIZACAO DOS MOTORES
        // =========================================

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

        // =========================================
        // INICIALIZACAO DO IMU
        // =========================================

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientacao =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                );

        imu.initialize(
                new IMU.Parameters(orientacao));

        imu.resetYaw();

        // =========================================
        // CALCULOS DA RODA
        // =========================================

        circunferenciaRoda =
                diametroRoda * Math.PI;

        ticksAlvo =
                (distanciaTeste / circunferenciaRoda)
                        * ticksPorVolta;

        telemetry.addLine("AION - TESTE AUTONOMO");
        telemetry.addData("Circunferencia", "%.2f cm",
                circunferenciaRoda);
        telemetry.addData("Ticks para distancia", "%.1f",
                ticksAlvo);
        telemetry.addLine("Aguardando START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            pararMotores();
            return;
        }

        anguloInicial = lerAngulo();

        // =========================================
        // ETAPA 1 - ANDAR PARA FRENTE
        // =========================================

        while (opModeIsActive()) {

            double ticksDir =
                    Math.abs(motorDir.getCurrentPosition());

            double ticksEsq =
                    Math.abs(motorEsq.getCurrentPosition());

            double ticksMedios =
                    (ticksDir + ticksEsq) / 2.0;

            distanciaPercorrida =
                    (ticksMedios / ticksPorVolta)
                            * circunferenciaRoda;

            anguloAtual = lerAngulo();

            erroAngular =
                    calcularErro(anguloInicial, anguloAtual);

            if (distanciaPercorrida >= distanciaTeste) {
                break;
            }

            // Calcula a correcao de direcao.
            double correcao =
                    Range.clip(
                            kpDirecao * erroAngular,
                            -0.12,
                            0.12
                    );

            // Ajuste de sinal deve ser validado
            // em teste supervisionado de baixa velocidade.
            double potenciaDir =
                    velocidadeBase - correcao;

            double potenciaEsq =
                    velocidadeBase + correcao;

            motorDir.setPower(
                    Range.clip(potenciaDir, -0.20, 0.20));

            motorEsq.setPower(
                    Range.clip(potenciaEsq, -0.20, 0.20));

            telemetry.addLine("ETAPA 1 - DESLOCAMENTO");
            telemetry.addData("Distancia", "%.1f / %.1f cm",
                    distanciaPercorrida, distanciaTeste);
            telemetry.addData("Angulo atual", "%.2f",
                    anguloAtual);
            telemetry.addData("Erro angular", "%.2f",
                    erroAngular);
            telemetry.update();

            idle();
        }

        pararMotores();

        if (!opModeIsActive()) {
            return;
        }

        sleep(5000);

        // =========================================
        // ETAPA 2 - GIRAR 90 GRAUS
        // =========================================

        double anguloAlvo =
                anguloInicial + 90.0;

        boolean giroConcluido =
                girarPara(anguloAlvo);

        if (!giroConcluido || !opModeIsActive()) {
            pararMotores();
            return;
        }

        sleep(5000);

        // =========================================
        // ETAPA 3 - VOLTAR A ORIENTACAO INICIAL
        // =========================================

        giroConcluido =
                girarPara(anguloInicial);

        pararMotores();

        telemetry.addLine("TESTE FINALIZADO");
        telemetry.addData("Distancia medida", "%.2f cm",
                distanciaPercorrida);
        telemetry.addData("Angulo final", "%.2f",
                lerAngulo());
        telemetry.addData("Retorno angular", giroConcluido);
        telemetry.update();
    }

    // =========================================
    // FUNCAO DE GIRO CONTROLADO PELO IMU
    // =========================================

    boolean girarPara(double alvo) {

        cronometro.reset();

        while (opModeIsActive()
                && cronometro.seconds() < tempoMaximoGiro) {

            anguloAtual = lerAngulo();

            erroAngular =
                    calcularErro(alvo, anguloAtual);

            if (Math.abs(erroAngular)
                    <= toleranciaAngular) {

                pararMotores();
                return true;
            }

            double potencia =
                    Range.clip(
                            kpGiro * Math.abs(erroAngular),
                            potenciaMinimaGiro,
                            potenciaMaximaGiro
                    );

            // O sentido precisa ser confirmado
            // com a orientacao real do robo.
            if (erroAngular > 0) {
                motorDir.setPower(potencia);
                motorEsq.setPower(-potencia);
            } else {
                motorDir.setPower(-potencia);
                motorEsq.setPower(potencia);
            }

            telemetry.addLine("GIRO PELO IMU");
            telemetry.addData("Alvo", "%.2f", alvo);
            telemetry.addData("Atual", "%.2f", anguloAtual);
            telemetry.addData("Erro", "%.2f", erroAngular);
            telemetry.update();

            idle();
        }

        pararMotores();

        return false;
    }

    // =========================================
    // LEITURA DO ANGULO
    // =========================================

    double lerAngulo() {

        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
    }

    // =========================================
    // NORMALIZACAO DO ERRO ANGULAR
    // =========================================

    double calcularErro(double alvo, double atual) {

        return ((alvo - atual + 540) % 360) - 180;
    }

    // =========================================
    // PARADA DOS MOTORES
    // =========================================

    void pararMotores() {

        motorDir.setPower(0);
        motorEsq.setPower(0);
    }
}