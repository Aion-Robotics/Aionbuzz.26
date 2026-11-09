package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Teste OpMode Autonomous 2")
public class OpMode2 extends OpMode {

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;
    private final ElapsedTime tempo = new ElapsedTime();
    // cronometro usado para medir o tempo de cada etapa
    private int Etapa = 0;
    // armazena a etapa atual da sequência autônoma

    private static final double POTENCIA_FRENTE = 0.50;
    private static final double POTENCIA_GIRO = 0.60;

    private static final double TEMPO_PRIMEIRO_LADO = 3.50;
    private static final double TEMPO_SEGUNDO_LADO = 2.60;
    private static final double TEMPO_TERCEIRO_LADO = 2.55;
    private static final double TEMPO_QUARTO_LADO = 2.50;

    private static final double TEMPO_GIRO = 0.39;
    private static final double TEMPO_PAUSA = 0.20;


    @Override
    public void init() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDir");

        motorEsquerdo.setDirection(DcMotor.Direction.REVERSE);
        // orientação do motor direito

        motorEsquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // faz o motor frear quando a potência for definida como zero

        parar();

        telemetry.addLine("Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        mudarEtapa(1);
    } // tbm zera o cronometro, fazendo que a etapa incie em zero

    @Override
    public void loop() {
        switch (Etapa) {
            // verifica a etapa atual e executa o comportamento referente a ela
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

                if (tempo.seconds() >= TEMPO_TERCEIRO_LADO) {
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
                    mudarEtapa(13);
                }
                break;

            case 13:
                parar();
                break;

            default:
                parar();
                break;
                // se a etapa possuir um valor não previsto, os motores são desligados por segurança
        }

        atualizarTelemetry();
    }

        @Override
        public void stop() {
            parar(); // desliga os dois motores no fim da etapa
        }

        private void andarFrente() {
            motorEsquerdo.setPower(POTENCIA_FRENTE);
            motorDireito.setPower(POTENCIA_FRENTE);
        }

        private void girarEsquerda() {
            motorEsquerdo.setPower(POTENCIA_GIRO);
            // motor esquerdo gira no sentido contrário
            motorDireito.setPower(-POTENCIA_GIRO);
        }
        private void parar(){
            motorEsquerdo.setPower(0.0);
            motorDireito.setPower(0.0);
            // zera a potência dos dois motores
        }

        private void mudarEtapa(int novaEtapa) {
            Etapa = novaEtapa;
            // atualiza etapa atual
            tempo.reset();
            // zera o cronometro para iniciar nova etapa apartir do zero
        }

    private void atualizarTelemetry(){
        telemetry.addData("Etapa", Etapa); // etapa atual executada
        telemetry.addData("Tempo da Etapa", "%.2f", tempo.seconds());
        // tempo que a etapa atual está ativa
        telemetry.addData("Motor Esquerdo", "%.2f", motorEsquerdo.getPower());
        // potência lógica do motor esquerdo
        telemetry.addData("Motor Direito", "%.2f", motorDireito.getPower());
        // potência lógica do motor direito
        telemetry.update();
        // atualiza os dados no Driver Hub
        // por ser um loop, os dados não se atualizam sozinhos sem esse método
    }
}