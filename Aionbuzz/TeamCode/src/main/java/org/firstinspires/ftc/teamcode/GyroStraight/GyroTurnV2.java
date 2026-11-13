package org.firstinspires.ftc.teamcode.GyroStraight;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous
public class GyroTurnV2 extends OpMode {

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private IMU imu;

    // Controle do ângulo contínuo
    private double anguloContinuo = 0;
    private double ultimoYaw = 0;

    // Controle da execução do giro no loop()
    private boolean turnoInicializado = false;
    private boolean turnoFinalizado = false;
    private boolean resultadoTurno = false;
    private boolean resultadoExibido = false;
    private long tempoFinalizacao;
    private boolean aguardandoFinalizacao = false;

    private double anguloAlvo;
    private double velocidadeAngularFiltrada;
    private double anguloAnterior;
    private long tempoAnterior;
    private int leiturasEstaveis;

    @Override
    public void init() {

        // Configuração dos motores
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");

        motorEsq.setDirection(DcMotor.Direction.REVERSE);

        motorDir.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        motorEsq.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Configuração do IMU
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientacao =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(
                new IMU.Parameters(orientacao));

        imu.resetYaw();

        telemetry.addLine("GyroTurnTeste inicializado");
        telemetry.addLine("Aguardando inicio...");
        telemetry.update();
    }

    @Override
    public void start() {

        // Inicializa a leitura do ângulo
        ultimoYaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        anguloContinuo = 0;

        // Executa somente uma curva de 90 graus
        turnoInicializado = false;
        turnoFinalizado = false;
        resultadoTurno = false;
        resultadoExibido = false;
    }

    @Override
    public void loop() {

        if (!turnoFinalizado) {
            resultadoTurno = gyroTurn(90);

            if (turnoFinalizado) {
                pararMotores();

                telemetry.addData("Resultado",
                        resultadoTurno ? "Curva concluida" : "Tempo esgotado");

                telemetry.addData("Angulo final", anguloContinuo);
                telemetry.update();

                tempoFinalizacao = System.nanoTime() + 2_000_000_000L;
                aguardandoFinalizacao = true;
                resultadoExibido = true;
            }
        } else if (resultadoExibido) {
            // Mantém o atraso final de 2000 ms sem bloquear o loop.
            if (aguardandoFinalizacao &&
                    System.nanoTime() >= tempoFinalizacao) {

                aguardandoFinalizacao = false;
                resultadoExibido = false;
                requestOpModeStop();
            }
        }
    }

    // Atualiza o ângulo acumulado do robô
    private double atualizarAngulo() {

        double yawAtual = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        double delta = yawAtual - ultimoYaw;

        // Corrige a passagem de +180 para -180 e vice-versa
        while (delta > 180) {
            delta -= 360;
        }

        while (delta < -180) {
            delta += 360;
        }

        anguloContinuo += delta;

        ultimoYaw = yawAtual;

        return anguloContinuo;
    }

    // Controlador PD para realizar a curva
    private boolean gyroTurn(double graus) {

        if (!turnoInicializado) {
            double anguloInicial = atualizarAngulo();

            anguloAlvo = anguloInicial + graus;

            // Filtro da velocidade angular
            velocidadeAngularFiltrada = 0;

            // Valores da leitura anterior
            anguloAnterior = anguloInicial;

            tempoAnterior = System.nanoTime();

            // Contador de estabilidade
            leiturasEstaveis = 0;

            resetRuntime();
            turnoInicializado = true;
        }

        if (getRuntime() < 12) {

            // Leitura do ângulo
            double anguloAtual = atualizarAngulo();

            // Calcula o erro angular
            double erro = anguloAlvo - anguloAtual;

            // Calcula o tempo entre leituras
            long tempoAtual = System.nanoTime();

            double deltaTempo =
                    (tempoAtual - tempoAnterior)
                            / 1_000_000_000.0;

            if (deltaTempo <= 0) {
                deltaTempo = 0.001;
            }

            // Calcula a velocidade angular
            double velocidadeAngular =
                    (anguloAtual - anguloAnterior) / deltaTempo;

            // Filtra a velocidade
            velocidadeAngularFiltrada =
                    0.8 * velocidadeAngularFiltrada
                            + 0.4 * velocidadeAngular;

            // Controlador PD
            double kpGiro = 0.018;
            double kdGiro = 0.00;

            double comando =
                    (kpGiro * erro)
                            - (kdGiro * velocidadeAngularFiltrada);

            // Limita a potência
            comando = Range.clip(comando, -0.35, 0.35);

            // Comanda os motores
            motorDir.setPower(-comando);
            motorEsq.setPower(comando);

            // Verifica a estabilidade
            if (Math.abs(erro) < 1 &&
                    Math.abs(velocidadeAngularFiltrada) < 4) {

                leiturasEstaveis++;

                if (leiturasEstaveis >= 5) {

                    pararMotores();
                    turnoFinalizado = true;

                    return true;
                }

            } else {
                leiturasEstaveis = 0;
            }

            // Telemetria
            telemetry.addData("Angulo atual", anguloAtual);
            telemetry.addData("Angulo alvo", anguloAlvo);
            telemetry.addData("Erro", erro);

            telemetry.addData(
                    "Velocidade bruta", velocidadeAngular);

            telemetry.addData(
                    "Velocidade filtrada",
                    velocidadeAngularFiltrada);

            telemetry.addData("Potencia", comando);

            telemetry.addData(
                    "Leituras estaveis", leiturasEstaveis);

            telemetry.addData(
                    "Tempo", getRuntime());

            telemetry.update();

            // Atualiza os valores anteriores
            anguloAnterior = anguloAtual;
            tempoAnterior = tempoAtual;

            return false;
        }

        // Interrompe os motores
        pararMotores();
        turnoFinalizado = true;

        return false;
    }

    // Para os motores
    private void pararMotores() {

        motorDir.setPower(0);
        motorEsq.setPower(0);

        motorDir.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        motorEsq.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
    }
}