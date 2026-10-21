package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Teste OpMode Autonomous 2")
public class OpMode2 extends OpMode {

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;
    private final ElapsedTime tempo = new ElapsedTime();
    private int Etapa = 0;

    private static final double POTENCIA_FRENTE = 0.50;
    private static final double POTENCIA_GIRO = 0.60;

    private static final double TEMPO_PRIMEIRO_LADO = 3.50;
    private static final double TEMPO_SEGUNDO_LADO = 2.60;
    private static final double TEMPO_TERCEIRO_LAD0 = 2.55;
    private static final double TEMPO_QUARTO_LADO = 2.50;

    private static final double TEMPO_GIRO = 0.39;
    private static final double TEMPO_PAUSA = 0.20;


    @Override
    public void init() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDir");

        motorDireito.setDirection(DcMotor.Direction.REVERSE);

        motorEsquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parar();

        telemetry.addLine("Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        mudarEtapa(1);
    }

    @Override
    public void loop() {
        switch (Etapa) {

            case 1:
                andarFrente();

                if (tempo.seconds() >= TEMPO_PRIMEIRO_LADO) {
                    parar();
                    mudarEtapa(2);
                }
                break;

            case 2:
                parar();

                if (tempo.seconds() >= TEMPO_PAUSA) {
                    mudarEtapa(3);
                }
                break;

            case 3:
                girarEsquerda();

                if (tempo.seconds() >= TEMPO_GIRO) {
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
                    mudarEtapa(7);
                }
                break;

            case 7:
                girarEsquerda();

                if (tempo.seconds() >= TEMPO_GIRO) {
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

                if (tempo.seconds() >= TEMPO_TERCEIRO_LAD0) {
                    parar();
                    mudarEtapa(10);
                }
                break;

            case 10:
                girarEsquerda();

                if (tempo.seconds() >= 0.46) {
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
                }
                break;

            default:
                parar();
                break;
        }

        atualizarTelemetry();
    }

    @Override
    public void stop() {
        parar();
    }

    private void andarFrente() {
        motorEsquerdo.setPower(POTENCIA_FRENTE);
        motorDireito.setPower(POTENCIA_FRENTE);
    }

    private void girarEsquerda() {
        motorEsquerdo.setPower(-POTENCIA_GIRO);
        motorDireito.setPower(POTENCIA_GIRO);
    }
    private void parar(){
        motorEsquerdo.setPower(0.0);
        motorDireito.setPower(0.0);
    }

    private void mudarEtapa(int novaEtapa) {
        Etapa = novaEtapa;
        tempo.reset();
    }

    private void atualizarTelemetry(){
        telemetry.addData("Etapa", Etapa);
        telemetry.addData("Tempo da Etapa", "%.2f", tempo.seconds());
        telemetry.addData("Motor Esquerdo", "%.2f", motorEsquerdo.getPower());
        telemetry.addData("Motor Direito", "%.2f", motorDireito.getPower());
        telemetry.update();
    }
}

