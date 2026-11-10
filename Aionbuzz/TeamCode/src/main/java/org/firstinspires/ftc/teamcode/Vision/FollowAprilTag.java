package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Autonomous
public class FollowAprilTag extends OpMode {

    private enum Estado {
        PROCURANDO_TAG,
        APROXIMANDO,
        EXECUTANDO_TAREFA,
        CONCLUIDO,
        FALHA
    }

    private Estado estadoAtual =
            Estado.PROCURANDO_TAG;

    private final AprilTagWebcam aprilTagWebcam =
            new AprilTagWebcam();

    private DcMotor motorEsq;
    private DcMotor motorDir;

    private final ElapsedTime tempoEstado =
            new ElapsedTime();

    private final ElapsedTime tempoBusca =
            new ElapsedTime();

    private static final int TAG_ALVO = 30;

    private static final double DISTANCIA_DESEJADA_CM =
            50.0;

    private static final double KP_DISTANCIA =
            0.010;

    private static final double KP_ANGULO =
            0.008;

    private static final double POTENCIA_MAXIMA =
            0.30;

    private static final double POTENCIA_MAXIMA_GIRO =
            0.20;

    private static final double TOLERANCIA_DISTANCIA_CM =
            3.0;

    private static final double TOLERANCIA_ANGULO_GRAUS =
            2.0;

    private static final double TIMEOUT_BUSCA_SEGUNDOS =
            5.0;

    private static final double TIMEOUT_TAREFA_SEGUNDOS =
            3.0;

    private static final double FILTRO =
            0.20;

    private double distanciaFiltrada;
    private double anguloFiltrado;

    @Override
    public void init() {
        motorEsq = hardwareMap.get(
                DcMotor.class,
                "motorEsq"
        );

        motorDir = hardwareMap.get(
                DcMotor.class,
                "motorDir"
        );


        motorEsq.setDirection(
                DcMotor.Direction.REVERSE
        );

        motorEsq.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        motorDir.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        pararMotores();

        aprilTagWebcam.init(
                hardwareMap,
                telemetry
        );

    }

    @Override
    public void start() {
        pararMotores();

        estadoAtual =
                Estado.PROCURANDO_TAG;

        tempoEstado.reset();
        tempoBusca.reset();

        distanciaFiltrada = 0.0;
        anguloFiltrado = 0.0;
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        switch (estadoAtual) {
            case PROCURANDO_TAG:
                procurarTag();
                break;

            case APROXIMANDO:
                aproximarDaTag();
                break;

            case EXECUTANDO_TAREFA:
                executarTarefaBioBuzz();
                break;

            case CONCLUIDO:
                pararMotores();
                telemetry.addLine(
                        "Autonomo concluido"
                );
                telemetry.update();
                break;

            case FALHA:
                pararMotores();
                telemetry.addLine(
                        "Autonomo interrompido por falha"
                );
                telemetry.update();
                break;
        }
    }

    private void procurarTag() {
        pararMotores();

        AprilTagDetection tag =
                aprilTagWebcam.getTagBySpecificId(
                        TAG_ALVO
                );

        telemetry.addLine(
                "Estado: procurando Tag 30"
        );
        telemetry.addData(
                "Tempo",
                "%.1f s",
                tempoBusca.seconds()
        );

        if (tag != null) {
            mudarEstado(
                    Estado.APROXIMANDO
            );
            telemetry.update();
            return;
        }

        if (tempoBusca.seconds()
                >= TIMEOUT_BUSCA_SEGUNDOS) {

            mudarEstado(Estado.FALHA);
            telemetry.addLine(
                    "Timeout procurando a Tag 30"
            );
        }

        telemetry.update();
    }

    private void aproximarDaTag() {
        AprilTagDetection tag =
                aprilTagWebcam.getTagBySpecificId(
                        TAG_ALVO
                );

        if (tag == null) {
            pararMotores();

            telemetry.addLine(
                    "Tag perdida durante aproximacao"
            );
            mudarEstado(Estado.FALHA);
            telemetry.update();
            return;
        }

        if (!(tag instanceof AprilTagSingleDetection)) {
            pararMotores();
            mudarEstado(Estado.FALHA);
            telemetry.addLine(
                    "Detecção invalida"
            );
            telemetry.update();
            return;
        }

        AprilTagSingleDetection singleDetection =
                (AprilTagSingleDetection) tag;

        if (singleDetection.metadata == null) {
            pararMotores();
            mudarEstado(Estado.FALHA);
            telemetry.addLine(
                    "Tag sem metadata"
            );
            telemetry.update();
            return;
        }

        double distanciaAtual =
                singleDetection.ftcPose.range;

        double anguloAtual =
                singleDetection.ftcPose.bearing;

        distanciaFiltrada =
                (1.0 - FILTRO)
                        * distanciaFiltrada
                        + FILTRO
                        * distanciaAtual;

        anguloFiltrado =
                (1.0 - FILTRO)
                        * anguloFiltrado
                        + FILTRO
                        * anguloAtual;

        double erroDistancia =
                distanciaFiltrada
                        - DISTANCIA_DESEJADA_CM;

        double erroAngulo =
                anguloFiltrado;

        double frente =
                KP_DISTANCIA
                        * erroDistancia;

        double giro =
                KP_ANGULO
                        * erroAngulo;

        frente = Range.clip(
                frente,
                -POTENCIA_MAXIMA,
                POTENCIA_MAXIMA
        );

        giro = Range.clip(
                giro,
                -POTENCIA_MAXIMA_GIRO,
                POTENCIA_MAXIMA_GIRO
        );

        if (Math.abs(erroDistancia)
                <= TOLERANCIA_DISTANCIA_CM) {
            frente = 0.0;
        }

        if (Math.abs(erroAngulo)
                <= TOLERANCIA_ANGULO_GRAUS) {
            giro = 0.0;
        }

        double esquerda =
                frente - giro;

        double direita =
                frente + giro;

        definirPotencia(
                esquerda,
                direita
        );

        boolean alinhado =
                Math.abs(erroDistancia)
                        <= TOLERANCIA_DISTANCIA_CM
                        &&
                        Math.abs(erroAngulo)
                                <= TOLERANCIA_ANGULO_GRAUS;

        telemetry.addLine(
                "Estado: aproximando"
        );
        telemetry.addData(
                "Distancia",
                "%.1f cm",
                distanciaFiltrada
        );
        telemetry.addData(
                "Angulo",
                "%.1f graus",
                anguloFiltrado
        );
        telemetry.addData(
                "Potencia esquerda",
                "%.2f",
                esquerda
        );
        telemetry.addData(
                "Potencia direita",
                "%.2f",
                direita
        );

        if (alinhado) {
            pararMotores();
            mudarEstado(
                    Estado.EXECUTANDO_TAREFA
            );
        }

        telemetry.update();
    }

    private void executarTarefaBioBuzz() {
        pararMotores();

        telemetry.addLine(
                "Estado: executando tarefa BioBuzz"
        );

        /*
         * Coloque aqui a tarefa específica do robô.
         *
         * Exemplos:
         * - acionar um servo;
         * - ligar um intake;
         * - levantar um mecanismo;
         * - depositar um elemento;
         * - aguardar um tempo de estabilização.
         */

        if (tempoEstado.seconds()
                >= TIMEOUT_TAREFA_SEGUNDOS) {

            mudarEstado(
                    Estado.CONCLUIDO
            );
        }

        telemetry.addData(
                "Tempo da tarefa",
                "%.1f s",
                tempoEstado.seconds()
        );
        telemetry.update();
    }

    private void mudarEstado(Estado novoEstado) {
        estadoAtual = novoEstado;
        tempoEstado.reset();
    }

    private void definirPotencia(
            double esquerda,
            double direita
    ) {
        motorEsq.setPower(
                Range.clip(esquerda, -1.0, 1.0)
        );

        motorDir.setPower(
                Range.clip(direita, -1.0, 1.0)
        );
    }

    private void pararMotores() {
        definirPotencia(0.0, 0.0);
    }

    @Override
    public void stop() {
        pararMotores();
        aprilTagWebcam.stop();
    }
}

