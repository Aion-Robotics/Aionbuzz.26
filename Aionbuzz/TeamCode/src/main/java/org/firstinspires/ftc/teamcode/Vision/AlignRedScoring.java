package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

public class AlignRedScoring extends OpMode {

    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotor motorEsq;
    private DcMotor motorDir;

    private enum Estado {
        Procurando_Cluster,
        Alinhando_e_Avancando,
        Concluido,
        Falha
    }

    private Estado estadoAtual = Estado.Procurando_Cluster;

    private static final String Cluster_Alvo = "Red_Scoring";

    private static final double DISTANCIA_DESEJADA_CM = 50.0;
    private static final double KP_DISTANCIA = 0.010;

    private static final double KP_ANGULO = 0.008;

    private static final double MAX_FRENTE = 0.25;

    private static final double MAX_GIRO = 0.18;
    private static final double TOLERANCIA_DISTANCIA_CM = 3.0;

    private static final double TOLERANCIA_ANGULO_GRAUS = 2.0;
    private static final double TIMEOUT_BUSCA_SEGUNDOS = 5.0;
    private static final double TIMEOUT_PERDA_SEGUNDOS = 0.5;

    private final ElapsedTime cronometroBusca = new ElapsedTime();

    private final ElapsedTime cronometroSemCluster = new ElapsedTime();

    @Override
    public void init() {
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");

        motorEsq.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pararMotores();

        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addLine("Alinhar com Red_Scoring");
        telemetry.addLine("Câmera e Motores Inicializados");
        telemetry.update();
    }

    @Override
    public void start() {
        pararMotores();

        estadoAtual = Estado.Procurando_Cluster;
        cronometroBusca.reset();
        cronometroSemCluster.reset();
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        switch (estadoAtual){
            case Procurando_Cluster:
                procurarCluster();
                break;

            case Alinhando_e_Avancando:
                alinharEAvancar();
                break;

            case Concluido:
                pararMotores();

                telemetry.addLine("Red_Scoring Alcançado");
                telemetry.update();
                break;

            case Falha:
                pararMotores();

                telemetry.addLine("Falha: Red_Scoring não encontrado");
                telemetry.update();
                break;
        }
    }

    private void procurarCluster(){
        pararMotores();

        AprilTagClusterDetection cluster = aprilTagWebcam.getClusterByName(Cluster_Alvo);
        telemetry.addLine("Estaado: Procurando cluster");
        telemetry.addData("Alvo", Cluster_Alvo);
        telemetry.addData("Tempo de Busca", "%.1f", cronometroBusca.seconds());

        if (cluster != null){
            cronometroSemCluster.reset();

            estadoAtual = Estado.Alinhando_e_Avancando;
            telemetry.addLine("Cluster encontrado");
        } else if (cronometroBusca.seconds() >= TIMEOUT_BUSCA_SEGUNDOS) {

            estadoAtual = Estado.Falha;
            telemetry.addLine("Timeout procurando cluster");
        }
        telemetry.update();
    }

    private void alinharEAvancar(){
        AprilTagClusterDetection cluster = aprilTagWebcam.getClusterByName(Cluster_Alvo);

        if (cluster == null){
            telemetry.addLine("Cluster perdido");

            if (cronometroSemCluster.seconds() >= TIMEOUT_BUSCA_SEGUNDOS){
                estadoAtual = Estado.Falha;
                telemetry.addLine("Movimento interrompido");
            }
            telemetry.update();
            return;
        }

        cronometroSemCluster.reset();

        double distanciaCm = cluster.ftcPose.range;
        double bearingGraus = cluster.ftcPose.bearing;

        double erroDistancia = distanciaCm - DISTANCIA_DESEJADA_CM;

        double erroAngulo = bearingGraus;

        double potenciaFrente = KP_DISTANCIA * erroDistancia;

        double potenciaGiro = KP_ANGULO * erroAngulo;

        potenciaFrente = Range.clip(
                potenciaFrente,
                -MAX_FRENTE,
                MAX_FRENTE
        );

        potenciaGiro = Range.clip(
                potenciaGiro,
                -MAX_GIRO,
                MAX_GIRO
        );

        if (Math.abs(erroDistancia)
                <= TOLERANCIA_DISTANCIA_CM) {
            potenciaFrente = 0.0;
        }

        if (Math.abs(erroAngulo)
                <= TOLERANCIA_ANGULO_GRAUS) {
            potenciaGiro = 0.0;
        }

        double potenciaEsq =
                potenciaFrente - potenciaGiro;

        double potenciaDir =
                potenciaFrente + potenciaGiro;

        potenciaEsq = Range.clip(
                potenciaEsq,
                -1.0,
                1.0
        );

        potenciaDir = Range.clip(
                potenciaDir,
                -1.0,
                1.0
        );

        boolean objetivoAlcancado = Math.abs(erroDistancia) <= TOLERANCIA_DISTANCIA_CM
                        && Math.abs(erroAngulo) <= TOLERANCIA_ANGULO_GRAUS;

        if (objetivoAlcancado) {
            pararMotores();

            estadoAtual =
                    Estado.Concluido;
        }

        else {
            definirPotencia(
                    potenciaEsq,
                    potenciaDir
            );
        }

        telemetry.addLine("Estado: alinhando e avancando");

        telemetry.addData("Cluster", cluster.metadata.shortName);

        telemetry.addData("Tags encontradas (%)", cluster.percentClusterFound);
        telemetry.addData("Distancia (cm)", "%.1f", distanciaCm);

        telemetry.addData("Bearing (graus)", "%.1f", bearingGraus);

        telemetry.addData("Erro distancia", "%.1f", erroDistancia);

        telemetry.addData("Erro angular", "%.1f", erroAngulo);

        telemetry.addData("Motor esquerdo", "%.2f", potenciaEsq);

        telemetry.addData("Motor direito", "%.2f", potenciaDir);

        telemetry.update();
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
