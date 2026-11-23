package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutonomoOpMode extends OpMode {
    public enum Etapa {
        ALINHAR_JARDIM,
        PAUSA_1,
        IR_PARA_JARDIM,
        PAUSA_2,
        DAR_RE,
        PAUSA_3,
        GIRO_90,
        PAUSA_4,
        IR_PARA_HIVE,
        PAUSA_5,
        GIRO_HIVE,
        PAUSA_6,
        POSICIONAR_NA_HIVE,
        PAUSA_7

    }
    private Etapa etapa;
    private final ElapsedTime timer = new ElapsedTime();

    // Hardware do Robô
    private DcMotor motorDir, motorEsq;
    private IMU imu;

    // Parâmetros do Robô
    private static final double DIAMETRO = 9.6;
    private static final double TICKS_POR_VOLTA = 537.6;
    private static final double CIRCUNFERENCIA_RODA =  DIAMETRO * Math.PI;
    private static final double POTENCIA_CORRECAO = 0.15;
    private static final double POTENCIA_GIRO = 0.35;
    private static final double POTENCIA_MOTORES = 1.0;

    // Variáveis de Angulação
    double anguloContinuo = 0.0;
    double ultimoYaw = 0.0;
    double anguloReferencial;

    // Parâmetros de Incialização
    boolean movimentoIniciado = false;
    private boolean giroIniciado = false;
    boolean curvaConcluida;
    boolean deslocamentoConcluido;

    // Variáveis do Giroscópio
    private double anguloAlvoGiro = 0.0;
    private double velocidadeAngularFiltrada = 0.0;
    private double tempoInicialGiro = 0.0;
    private double anguloAnteriorGiro = 0.0;
    private int leiturasEstaveis = 0;

    @Override
    public void init() {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        imu = hardwareMap.get(IMU.class, "IMU");

        // Reverte o sentido de um dos motores da base
        motorEsq.setDirection(DcMotor.Direction.REVERSE);

        // Registra a orientação do Control Hub enquanto montado
        RevHubOrientationOnRobot orientacao = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        imu.initialize(new IMU.Parameters(orientacao));
        imu.resetYaw();

        // Inicia as etapas
        mudarEtapa(Etapa.ALINHAR_JARDIM);
    }

    @Override
    public void loop() {
        double tempoDecorrido = timer.milliseconds();
        switch (etapa){
            // Alinha o robô na direção do Jardim
            case ALINHAR_JARDIM:
                curvaConcluida = girarEixo(90); // Gira no sentido Anti-Horário [PARA ESQUERDA]

                if (curvaConcluida)
                    mudarEtapa(Etapa.PAUSA_1);
                break;

            case PAUSA_1:
                if (tempoDecorrido >= 500) // Pausa por 0.5 segundos
                    mudarEtapa(Etapa.IR_PARA_JARDIM);
                break;

            case IR_PARA_JARDIM: // Inicia o movimento até o Jardim
                deslocamentoConcluido = andarDistancia(100); // 1 metro

                if (deslocamentoConcluido)
                    mudarEtapa(Etapa.PAUSA_2);
                break;

            case PAUSA_2:
                if (tempoDecorrido >= 300) // Pausa por 0.3 segundos
                    mudarEtapa(Etapa.DAR_RE);
                break;

            case DAR_RE: // Anda 88 cm para trás
                deslocamentoConcluido = andarDistancia(-88);

                if (deslocamentoConcluido)
                    mudarEtapa(Etapa.PAUSA_3);
                break;

            case PAUSA_3:
                if (tempoDecorrido >= 300) // Pausa por 0.3 segundos
                    mudarEtapa(Etapa.GIRO_90);
                break;

            case GIRO_90:
                // Gira no sentido Horário [PARA DIREITA]
                curvaConcluida = girarEixo(-90);

                if (curvaConcluida)
                    mudarEtapa(Etapa.PAUSA_4);
                break;

            case PAUSA_4:
                if (tempoDecorrido >= 500) // Pausa por 0.5 segundos
                    mudarEtapa(Etapa.IR_PARA_HIVE);
                break;

            case IR_PARA_HIVE: // Inicia o movimento até a HIVE
                deslocamentoConcluido = andarDistancia(235); // 2.35 metros

                if (deslocamentoConcluido)
                    mudarEtapa(Etapa.PAUSA_5);
                break;

            case PAUSA_5:
                if (tempoDecorrido >= 300) // Pausa por 0.3 segundos
                    mudarEtapa(Etapa.GIRO_HIVE);
                break;

            case GIRO_HIVE:
                curvaConcluida = girarEixo(-90); // Gira no sentido horário [PARA DIREITA]

                if (curvaConcluida)
                    mudarEtapa(Etapa.PAUSA_6);
                break;

            case PAUSA_6:
                if (tempoDecorrido >= 500) // Pausa por 0.5 segundos
                    mudarEtapa(Etapa.POSICIONAR_NA_HIVE);
                break;

            case POSICIONAR_NA_HIVE: // Anda 64 cm se posicionando à frente da HIVE
                deslocamentoConcluido = andarDistancia(64);

                if(deslocamentoConcluido)
                    mudarEtapa(Etapa.PAUSA_7);
                break;

            case PAUSA_7:
                if (tempoDecorrido >= 300) // Pausa por 0.3 segundos
                    pararMotores(); // para os motores
                break;
        }
    }
    private void mudarEtapa(Etapa novaEtapa){
        etapa = novaEtapa;
        timer.reset();
    }
    private double atualizarAngulo() {
        double yawAtual = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        double delta = yawAtual - ultimoYaw;
        delta = AngleUnit.normalizeDegrees(delta);

        anguloContinuo += delta;
        ultimoYaw = yawAtual;

        return anguloContinuo;
    }

    private boolean andarDistancia(double distanciaCm) {
        double tolerancia = 0.5; // Tolerância de 0.5 cm

        // Recebe o sentido do movimento
        double direcao = Math.signum(distanciaCm);
        double alvoAbsoluto = Math.abs(distanciaCm);

        // Verifica se o movimento já foi iniciado
        if (!movimentoIniciado) {
            // Calibra os Encoders
            motorDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorEsq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            anguloReferencial = atualizarAngulo();
            movimentoIniciado = true;
            resetRuntime();
        }

        // 2. Verifica se atingiu o tempo limite de 30 segundos
        if (getRuntime() >= 30.0) {
            pararMotores();
            movimentoIniciado = false; // Reseta para um próximo uso
            return true;
        }

        // 3. Calcula a distância percorrida
        double ticksDir = motorDir.getCurrentPosition();
        double ticksEsq = motorEsq.getCurrentPosition();

        double ticksPercorridos = (Math.abs(ticksDir) + Math.abs(ticksEsq)) / 2.0;
        double distanciaPercorrida = (ticksPercorridos / TICKS_POR_VOLTA) * CIRCUNFERENCIA_RODA;
        double distanciaRestante = alvoAbsoluto - distanciaPercorrida;

        boolean dentroDoLimite = (distanciaRestante <= tolerancia);

        // Verifica se chegou ao destino
        if (dentroDoLimite) {
            pararMotores();
            movimentoIniciado = false; // Reseta para um próximo uso
            return true;
        }

        // 5. Cálculo da correção PID e potências
        double anguloAtual = atualizarAngulo();
        double desvio = anguloAtual - anguloReferencial;

        double kpDirecao = -0.012;
        double correcao = Range.clip(desvio * kpDirecao, -POTENCIA_CORRECAO, POTENCIA_CORRECAO);

        double velocidadeBase = Math.min(0.30, POTENCIA_CORRECAO * (1.0 + (distanciaRestante / 30.0)));
        double velocidade = velocidadeBase * direcao;

        double potenciaDir = Range.clip(velocidade - correcao, -POTENCIA_MOTORES, POTENCIA_MOTORES);
        double potenciaEsq = Range.clip(velocidade + correcao, -POTENCIA_MOTORES, POTENCIA_MOTORES);

        motorEsq.setPower(potenciaEsq);
        motorDir.setPower(potenciaDir);

        // Telemetria
        telemetry.addData("Distância Percorrida", distanciaPercorrida * direcao);
        telemetry.addData("Distância Restante", distanciaRestante);
        telemetry.addData("Desvio", desvio);
        telemetry.addData("Ângulo Atual", anguloAtual);
        telemetry.addData("Ângulo Inicial", anguloReferencial);

        return false; // O movimento ainda não foi concluido
    }
    private boolean girarEixo(double graus) {
        // Verifica se o giro já foi iniciado
        if (!giroIniciado) {
            double anguloInicial = atualizarAngulo();
            anguloAlvoGiro = anguloInicial + graus;
            velocidadeAngularFiltrada = 0.0;

            resetRuntime();
            timer.reset();
            tempoInicialGiro = timer.seconds();

            anguloAnteriorGiro = anguloInicial;
            leiturasEstaveis = 0;
            giroIniciado = true;
        }

        // Verifica timeout de 12s
        if (getRuntime() >= 12.0) {
            pararMotores();
            giroIniciado = false;
            return true;
        }

        double kpGiro = 0.016;
        double tempoAtual = timer.seconds();
        double anguloAtual = atualizarAngulo();
        double erro = anguloAlvoGiro - anguloAtual;

        // Calcula o intervalo de tempo
        double deltaTempo = tempoAtual - tempoInicialGiro;

        // Impede uma divisão por 0
        if (deltaTempo <= 0)
            deltaTempo = 0.001;

        // Calcula a taxa de variação do ângulo (velocidade angular)
        double velocidadeAngular = (anguloAtual - anguloAnteriorGiro) / deltaTempo;
        velocidadeAngularFiltrada = (0.6 * velocidadeAngularFiltrada) + (0.4 * velocidadeAngular);

        double comando = Range.clip(kpGiro * erro, -POTENCIA_GIRO, POTENCIA_GIRO);

        motorDir.setPower(-comando);
        motorEsq.setPower(comando);

        if (Math.abs(erro) < 1.0 && Math.abs(velocidadeAngularFiltrada) < 4) {
            leiturasEstaveis++;

            if (leiturasEstaveis >= 5) {
                pararMotores();
                giroIniciado = false;
                return true;
            }
        } else {
            leiturasEstaveis = 0;
        }

        // Telemetria
        telemetry.addData("Ângulo atual", anguloAtual);
        telemetry.addData("Ângulo alvo", anguloAlvoGiro);
        telemetry.addData("Erro", erro);
        telemetry.addData("Velocidade bruta", velocidadeAngular);
        telemetry.addData("Velocidade filtrada", velocidadeAngularFiltrada);
        telemetry.addData("Leituras estaveis", leiturasEstaveis);
        telemetry.addData("Tempo", getRuntime());

        // Atualiza os valores anteriores
        anguloAnteriorGiro = anguloAtual;
        tempoInicialGiro = tempoAtual;

        return false;
    }
    private void pararMotores(){
        // Zera a Potência dos Motores
        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);

        // Trava o robô no lugar
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
