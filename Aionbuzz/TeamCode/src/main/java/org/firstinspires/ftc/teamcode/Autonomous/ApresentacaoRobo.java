package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class ApresentacaoRobo extends OpMode {

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;

    private final ElapsedTime tempo = new ElapsedTime();


    private enum Estado {
        INICIO,

        FRENTE_IDA,
        FRENTE_PAUSA,
        FRENTE_VOLTA,
        FRENTE_FINAL,

        DIREITA_GIRO_IDA,
        DIREITA_IDA,
        DIREITA_ALINHAR,
        DIREITA_VOLTA,
        DIREITA_FINAL,

        TRAS_IDA,
        TRAS_PAUSA,
        TRAS_VOLTA,
        TRAS_FINAL,

        ESQUERDA_GIRO_IDA,
        ESQUERDA_IDA,
        ESQUERDA_ALINHAR,
        ESQUERDA_VOLTA,
        ESQUERDA_FINAL,

        CURVA_ESQUERDA,
        CURVA_PAUSA,
        CURVA_DIREITA,
        CURVA_FINAL,

        GIRO_360,
        GIRO_FINAL,

        DANCA_FRENTE,
        DANCA_PAUSA_1,
        DANCA_TRAS,
        DANCA_PAUSA_2,
        DANCA_GIRO_ESQUERDA,
        DANCA_PAUSA_3,
        DANCA_GIRO_DIREITA,
        DANCA_PAUSA_4,
        DANCA_GIRO_ESQUERDA_FINAL,

        FINAL
    }

    private Estado estadoAtual = Estado.INICIO;

    private static final double POTENCIA_FRENTE = 0.30;
    private static final double POTENCIA_GIRO = 0.30;

    private static final double POTENCIA_CURVA_INTERNA = 0.14;
    private static final double POTENCIA_CURVA_EXTERNA = 0.30;

    private static final double TEMPO_PAUSA = 0.35;
    private static final double TEMPO_DESLOCAMENTO = 0.70;
    private static final double TEMPO_GIRO_90 = 0.30;
    private static final double TEMPO_CURVA = 0.70;
    private static final double TEMPO_GIRO_360 = 1.20;

    private static final double TEMPO_DANCA_FRENTE = 0.25;
    private static final double TEMPO_DANCA_TRAS = 0.25;
    private static final double TEMPO_DANCA_GIRO_CURTO = 0.20;


    @Override
    public void init() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDir");

        motorDireito.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parar();

        telemetry.addLine("Apresentação Pronta");
        telemetry.addLine("Aguardando Start");
        telemetry.update();
    }

    @Override
    public void start() {
        mudarEstado(Estado.INICIO);
    }

    @Override
    public void loop() {
        switch (estadoAtual) {

            case INICIO:
                parar();
                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.FRENTE_IDA);
                }
                break;

            case FRENTE_IDA:
                andarFrente();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.FRENTE_PAUSA);
                }
                break;

            case FRENTE_PAUSA:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.FRENTE_VOLTA);
                }
                break;

            case FRENTE_VOLTA:
                andarTras();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.FRENTE_FINAL);
                }
                break;

            case FRENTE_FINAL:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.DIREITA_GIRO_IDA);
                }
                break;

            case DIREITA_GIRO_IDA:
                girarDireita();

                if (tempoAtingiu(TEMPO_GIRO_90)) {
                    parar();
                    mudarEstado(Estado.DIREITA_IDA);
                }
                break;

            case DIREITA_IDA:
                andarFrente();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.DIREITA_ALINHAR);
                }
                break;

            case DIREITA_ALINHAR:
                girarEsquerda();

                if (tempoAtingiu(TEMPO_GIRO_90)) {
                    parar();
                    mudarEstado(Estado.DIREITA_VOLTA);
                }
                break;

            case DIREITA_VOLTA:
                andarTras();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.DIREITA_FINAL);
                }
                break;

            case DIREITA_FINAL:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.TRAS_IDA);
                }
                break;

            case TRAS_IDA:
                andarTras();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.TRAS_PAUSA);
                }
                break;

            case TRAS_PAUSA:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.TRAS_VOLTA);
                }
                break;

            case TRAS_VOLTA:
                andarFrente();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.TRAS_FINAL);
                }
                break;

            case TRAS_FINAL:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.ESQUERDA_GIRO_IDA);
                }
                break;

            case ESQUERDA_GIRO_IDA:
                girarEsquerda();

                if (tempoAtingiu(TEMPO_GIRO_90)) {
                    parar();
                    mudarEstado(Estado.ESQUERDA_IDA);
            }
                break;

            case ESQUERDA_IDA:
                andarFrente();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.ESQUERDA_ALINHAR);
                }
                break;

            case ESQUERDA_ALINHAR:
                girarDireita();

                if (tempoAtingiu(TEMPO_GIRO_90)) {
                    parar();
                    mudarEstado(Estado.ESQUERDA_VOLTA);
                }
                break;

            case ESQUERDA_VOLTA:
                andarTras();

                if (tempoAtingiu(TEMPO_DESLOCAMENTO)) {
                    parar();
                    mudarEstado(Estado.ESQUERDA_FINAL);
                }
                break;

            case ESQUERDA_FINAL:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.CURVA_ESQUERDA);
                }
                break;

            case CURVA_ESQUERDA:
                curvaEsquerda();

                if (tempoAtingiu(TEMPO_CURVA)) {
                    parar();
                    mudarEstado(Estado.CURVA_PAUSA);
                }
                break;

            case CURVA_PAUSA:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.CURVA_DIREITA);
                }
                break;

            case CURVA_DIREITA:
                curvaDireita();

                if (tempoAtingiu(TEMPO_CURVA)) {
                    parar();
                    mudarEstado(Estado.CURVA_FINAL);
                }
                break;

            case CURVA_FINAL:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.GIRO_360);
                }
                break;

            case GIRO_360:
                girarEsquerda();

                if (tempoAtingiu(TEMPO_GIRO_360)) {
                    parar();
                    mudarEstado(Estado.GIRO_FINAL);
                }
                break;

            case GIRO_FINAL:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.DANCA_FRENTE);
                }
                break;

            case DANCA_FRENTE:
                andarFrente();

                if (tempoAtingiu(TEMPO_DANCA_FRENTE)) {
                    parar();
                    mudarEstado(Estado.DANCA_PAUSA_1);
                }
                break;

            case DANCA_PAUSA_1:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.DANCA_TRAS);
                }
                break;

            case DANCA_TRAS:
                andarTras();

                if (tempoAtingiu(TEMPO_DANCA_TRAS)) {
                    parar();
                    mudarEstado(Estado.DANCA_PAUSA_2);
                }
                break;

            case DANCA_PAUSA_2:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.DANCA_GIRO_ESQUERDA);
                }
                break;

            case DANCA_GIRO_ESQUERDA:
                girarEsquerda();

                if (tempoAtingiu(TEMPO_DANCA_GIRO_CURTO)) {
                    parar();
                    mudarEstado(Estado.DANCA_PAUSA_3);
                }
                break;

            case DANCA_PAUSA_3:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.DANCA_GIRO_DIREITA);
                }
                break;

            case DANCA_GIRO_DIREITA:
                girarDireita();

                if (tempoAtingiu(TEMPO_DANCA_GIRO_CURTO * 2)) {
                    parar();
                    mudarEstado(Estado.DANCA_PAUSA_4);
                }
                break;

            case DANCA_PAUSA_4:
                parar();

                if (tempoAtingiu(TEMPO_PAUSA)) {
                    mudarEstado(Estado.DANCA_GIRO_ESQUERDA_FINAL);
                }
                break;

            case DANCA_GIRO_ESQUERDA_FINAL:
                girarEsquerda();

                if (tempoAtingiu(TEMPO_DANCA_GIRO_CURTO)) {
                    parar();
                    mudarEstado(Estado.FINAL);
                }
                break;

            case FINAL:
                parar();
                break;

            default:
                parar();
                mudarEstado(Estado.FINAL);
                break;
        }

        atualizarTelemetria();
    }

    @Override
    public void stop() {
        parar();
    }

    private void andarFrente() {
        motorEsquerdo.setPower(POTENCIA_FRENTE);
        motorDireito.setPower(POTENCIA_FRENTE);
    }

    private void andarTras() {
        motorEsquerdo.setPower(-POTENCIA_FRENTE);
        motorDireito.setPower(-POTENCIA_FRENTE);
    }

    private void girarEsquerda() {
        motorEsquerdo.setPower(-POTENCIA_GIRO);
        motorDireito.setPower(POTENCIA_GIRO);
    }

    private void girarDireita() {
        motorEsquerdo.setPower(POTENCIA_GIRO);
        motorDireito.setPower(-POTENCIA_GIRO);
    }

    private void curvaEsquerda() {
        motorEsquerdo.setPower(POTENCIA_CURVA_INTERNA);
        motorDireito.setPower(POTENCIA_CURVA_EXTERNA);
    }

    private void curvaDireita() {
        motorEsquerdo.setPower(POTENCIA_CURVA_EXTERNA);
        motorDireito.setPower(POTENCIA_CURVA_INTERNA);
    }

    private void parar() {
        motorEsquerdo.setPower(0);
        motorDireito.setPower(0);
    }

    private void mudarEstado(Estado novoEstado) {
        estadoAtual = novoEstado;
        tempo.reset();
    }

    private boolean tempoAtingiu(double tempoLimite) {
        return tempo.seconds() >= tempoLimite;
    }

    private void atualizarTelemetria() {
        telemetry.addData("Estado", estadoAtual);
        telemetry.addData("Tempo", "%.2f s", tempo.seconds());
        telemetry.addData(
                "Motor esquerdo",
                "%.2f",
                motorEsquerdo.getPower()
        );
        telemetry.addData(
                "Motor direito",
                "%.2f",
                motorDireito.getPower()
        );
        telemetry.update();
    }
}
