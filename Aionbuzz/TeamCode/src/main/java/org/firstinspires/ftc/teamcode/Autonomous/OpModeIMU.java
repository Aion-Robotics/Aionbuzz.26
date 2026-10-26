package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private double alvoheading = 0.0;
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
        switch (etapa);

        case 1:
            andarFrente();

            if (tempo.seconds() >= TEMPO_PRIMEIRO_LADO) {
                parar();
                mudarEtapa(2);
            }

            break;

        case 2:
            parar();

            if (tempo.seconds() >= TEMPO_PAUSA);

            iniciarGiroEsquerda(3)

    }

    @Override
    public void stop() {
        parar(); // desliga os dois motores no fim da etapa
    }

    private void andarFrente() {
        motorEsquerdo.setPower(POTENCIA_FRENTE);
        motorDireito.setPower(POTENCIA_FRENTE);
    }

    private void parar(){
        motorEsquerdo.setPower(0.0);
        motorDireito.setPower(0.0);
        // zera a potência dos dois motores
    }

    private void mudarEtapa(int novaEtapa) {
        etapa = novaEtapa;
        // atualiza etapa atual
        tempo.reset();
        // zera o cronometro para iniciar nova etapa apartir do zero
    }
}
