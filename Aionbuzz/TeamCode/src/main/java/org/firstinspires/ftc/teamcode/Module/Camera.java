package org.firstinspires.ftc.teamcode.Module;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;      
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;


@TeleOp(name = "Diagnóstico Câmera 2")
public class Camera extends OpMode {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final int CICLOS_PARA_ESTABILIZAR = 3;

    /*
     * Este robô possui somente as duas hives vermelhas disponíveis.
     * As opções azuis ficam desativadas até existirem tags corretas.
     */
    private static final boolean FILTRAR_CLUSTERS_POR_METADATA = true;

    private static final String META_HIVE_UP = "RED SCORING";
    private static final String META_HIVE_RIGHT = "RED AUDIENCE";
    private static final String META_HIVE_DOWN = "";
    private static final String META_HIVE_LEFT = "";

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private int[] tagsDaHive = {30, 31, 32, 33};
    private String nomeHive = "Vermelha - Lado Oposto da Audiencia";
    private String metadataHiveSelecionada = META_HIVE_UP;

    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    private int ciclosComAlvoValido;
    private int ciclosSemAlvoValido;
    private long ultimoAlvoValidoMs;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Diagnostico de camera inicializado");
        telemetry.addData("Camera", "Webcam 1");
        telemetry.addData("Resolucao solicitada", "%d x %d",
                CAMERA_WIDTH, CAMERA_HEIGHT);
        telemetry.addData("Filtro por metadata", "%s",
                FILTRAR_CLUSTERS_POR_METADATA ? "ATIVO" : "DESATIVADO");
        telemetry.addLine("Nenhum mecanismo sera acionado");
        telemetry.addLine("D-pad: seleciona a hive");
        telemetry.update();
    }

    @Override
    public void loop() {
        selecionarHive();

        ArrayList<AprilTagDetection> deteccoes =
                aprilTagProcessor.getDetections();

        AprilTagDetection melhorAlvo = null;
        double menorRange = Double.POSITIVE_INFINITY;
        boolean temPoseValida = false;

        telemetry.addData("Deteccoes recebidas", deteccoes.size());

        for (int i = 0; i < deteccoes.size(); i++) {
            AprilTagDetection deteccao = deteccoes.get(i);
            boolean poseValida = deteccao.ftcPose != null;

            if (poseValida) {
                temPoseValida = true;
            }

            telemetry.addData(
                    "Tag " + (i + 1),
                    "%s | pose=%s",
                    obterIdentificacao(deteccao),
                    poseValida ? "VALIDA" : "NULA");

            if (deteccao instanceof AprilTagClusterDetection) {
                exibirMetadataDoCluster(
                        (AprilTagClusterDetection) deteccao,
                        i + 1);
            }

            if (poseValida) {
                telemetry.addData(
                        "Pose " + (i + 1),
                        "range=%.1f in | bearing=%.1f | yaw=%.1f",
                        deteccao.ftcPose.range,
                        deteccao.ftcPose.bearing,
                        deteccao.ftcPose.yaw);
            }

            if (poseValida
                    && deteccaoPertenceAHive(deteccao)
                    && deteccao.ftcPose.range < menorRange) {
                menorRange = deteccao.ftcPose.range;
                melhorAlvo = deteccao;
            }
        }

        atualizarEstabilidade(melhorAlvo != null);

        boolean alvoEstavel =
                ciclosComAlvoValido >= CICLOS_PARA_ESTABILIZAR;

        telemetry.addData("Portal", visionPortal.getCameraState());
        telemetry.addData("Resolucao", "%d x %d",
                CAMERA_WIDTH, CAMERA_HEIGHT);
        telemetry.addData("Hive selecionada", nomeHive);
        telemetry.addData("Metadata esperada", metadataHiveSelecionada);
        telemetry.addData("Deteccao bruta",
                deteccoes.isEmpty() ? "NAO" : "SIM");
        telemetry.addData("Pose valida em alguma deteccao",
                temPoseValida ? "SIM" : "NAO");
        telemetry.addData("Alvo valido da hive",
                melhorAlvo == null ? "NAO" : "SIM");
        telemetry.addData("Ciclos de alvo validos",
                ciclosComAlvoValido);
        telemetry.addData("Ciclos sem alvo valido",
                ciclosSemAlvoValido);
        telemetry.addData("Estado do alvo",
                alvoEstavel ? "ESTAVEL" : "INSTAVEL");

        if (ultimoAlvoValidoMs > 0) {
            telemetry.addData("Tempo desde ultimo alvo",
                    "%d ms", System.currentTimeMillis() - ultimoAlvoValidoMs);
        }

        if (melhorAlvo == null) {
            telemetry.addLine("Alvo da hive: nenhum com pose e metadata/ID validos");
        } else {
            telemetry.addData("Alvo da hive",
                    obterIdentificacao(melhorAlvo));
            telemetry.addData("Alvo range",
                    "%.1f in", melhorAlvo.ftcPose.range);
            telemetry.addData("Alvo bearing",
                    "%.1f", melhorAlvo.ftcPose.bearing);
        }

        telemetry.update();
    }

    private void exibirMetadataDoCluster(
            AprilTagClusterDetection cluster,
            int numero) {
        if (cluster.metadata == null) {
            telemetry.addData(
                    "Cluster " + numero + " metadata",
                    "SEM METADATA | %d%% encontrado",
                    cluster.percentClusterFound);
        } else {
            telemetry.addData(
                    "Cluster " + numero + " metadata",
                    "%s | %d%% encontrado",
                    cluster.metadata.name,
                    cluster.percentClusterFound);
        }
    }

    private void atualizarEstabilidade(boolean temAlvoValido) {
        if (temAlvoValido) {
            ciclosComAlvoValido++;
            ciclosSemAlvoValido = 0;
            ultimoAlvoValidoMs = System.currentTimeMillis();
        } else {
            ciclosComAlvoValido = 0;
            ciclosSemAlvoValido++;
        }
    }

    private void selecionarHive() {
        if (gamepad1.dpad_up && !ultimoUp) {
            tagsDaHive = new int[]{30, 31, 32, 33};
            nomeHive = "Vermelha - Lado Oposto da Audiencia";
            metadataHiveSelecionada = META_HIVE_UP;
        }
        if (gamepad1.dpad_right && !ultimoRight) {
            tagsDaHive = new int[]{34, 35, 36, 37};
            nomeHive = "Vermelha - Lado da Audiencia";
            metadataHiveSelecionada = META_HIVE_RIGHT;
        }
        if (gamepad1.dpad_down && !ultimoDown) {
            tagsDaHive = new int[0];
            nomeHive = "INDISPONIVEL - tags azuis incorretas";
            metadataHiveSelecionada = META_HIVE_DOWN;
        }
        if (gamepad1.dpad_left && !ultimoLeft) {
            tagsDaHive = new int[0];
            nomeHive = "INDISPONIVEL - tags azuis incorretas";
            metadataHiveSelecionada = META_HIVE_LEFT;
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private boolean deteccaoPertenceAHive(AprilTagDetection deteccao) {
        if (deteccao instanceof AprilTagSingleDetection) {
            int id = ((AprilTagSingleDetection) deteccao).id;
            return idPertenceAHive(id);
        }

        if (deteccao instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) deteccao;

            if (!FILTRAR_CLUSTERS_POR_METADATA) {
                return true;
            }

            return cluster.metadata != null
                    && cluster.metadata.name.equals(metadataHiveSelecionada);
        }

        return false;
    }

    private boolean idPertenceAHive(int id) {
        for (int idPermitido : tagsDaHive) {
            if (idPermitido == id) {
                return true;
            }
        }
        return false;
    }

    private String obterIdentificacao(AprilTagDetection deteccao) {
        if (deteccao instanceof AprilTagSingleDetection) {
            return "ID=" + ((AprilTagSingleDetection) deteccao).id;
        }
        if (deteccao instanceof AprilTagClusterDetection) {
            return "CLUSTER/HIVE";
        }
        return "TIPO DESCONHECIDO";
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
