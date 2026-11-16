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

@TeleOp(name = "Diagnóstico Câmera")
public class AprilTagCamera extends OpMode {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Quantidade de ciclos válidos necessária para considerar a detecção estável.
    private static final int CICLOS_PARA_ESTABILIZAR = 3;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private int[] tagsDaHive = {30, 31, 32, 33};
    private String nomeHive = "Vermelha - Lado Oposto da Audiencia";

    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    private int ciclosComPoseValida;
    private int ciclosSemPoseValida;
    private long ultimaPoseValidaMs;

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

        telemetry.addLine("AprilTag Camera inicializada");
        telemetry.addData("Camera", "Webcam 1");
        telemetry.addData("Resolucao solicitada", "%d x %d",
                CAMERA_WIDTH, CAMERA_HEIGHT);
        telemetry.addLine("Nenhum mecanismo sera acionado");
        telemetry.addLine("D-pad: seleciona a hive");
        telemetry.update();
    }

    @Override
    public void loop() {
        selecionarHive();

        ArrayList<AprilTagDetection> deteccoes =
                aprilTagProcessor.getDetections();

        boolean temDeteccaoBruta = !deteccoes.isEmpty();
        boolean temPoseValida = false;
        AprilTagDetection melhorAlvo = null;
        double menorRange = Double.POSITIVE_INFINITY;

        for (int i = 0; i < deteccoes.size(); i++) {
            AprilTagDetection deteccao = deteccoes.get(i);
            boolean poseValida = deteccao.ftcPose != null;

            if (poseValida) {
                temPoseValida = true;

                if (deteccaoPertenceAHive(deteccao)
                        && deteccao.ftcPose.range < menorRange) {
                    menorRange = deteccao.ftcPose.range;
                    melhorAlvo = deteccao;
                }
            }

            telemetry.addData(
                    "Tag " + (i + 1),
                    "%s | pose=%s",
                    obterIdentificacao(deteccao),
                    poseValida ? "VALIDA" : "NULA");

            if (poseValida) {
                telemetry.addData(
                        "Pose " + (i + 1),
                        "range=%.1f in | bearing=%.1f | yaw=%.1f",
                        deteccao.ftcPose.range,
                        deteccao.ftcPose.bearing,
                        deteccao.ftcPose.yaw);
            }
        }

        atualizarEstabilidade(temPoseValida);

        boolean deteccaoEstavel =
                ciclosComPoseValida >= CICLOS_PARA_ESTABILIZAR;

        telemetry.addData("Portal", visionPortal.getCameraState());
        telemetry.addData("Resolucao", "%d x %d",
                CAMERA_WIDTH, CAMERA_HEIGHT);
        telemetry.addData("Hive selecionada", nomeHive);
        telemetry.addData("Deteccao bruta", temDeteccaoBruta ? "SIM" : "NAO");
        telemetry.addData("Pose valida", temPoseValida ? "SIM" : "NAO");
        telemetry.addData("Ciclos validos consecutivos",
                ciclosComPoseValida);
        telemetry.addData("Ciclos sem pose consecutivos",
                ciclosSemPoseValida);
        telemetry.addData("Estado",
                deteccaoEstavel ? "ESTAVEL" : "INSTAVEL");

        if (ultimaPoseValidaMs > 0) {
            telemetry.addData("Tempo desde ultima pose",
                    "%d ms", System.currentTimeMillis() - ultimaPoseValidaMs);
        }

        if (melhorAlvo == null) {
            telemetry.addLine("Alvo da hive: nenhum com pose valida");
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

    private void atualizarEstabilidade(boolean temPoseValida) {
        if (temPoseValida) {
            ciclosComPoseValida++;
            ciclosSemPoseValida = 0;
            ultimaPoseValidaMs = System.currentTimeMillis();
        } else {
            ciclosComPoseValida = 0;
            ciclosSemPoseValida++;
        }
    }

    private void selecionarHive() {
        if (gamepad1.dpad_up && !ultimoUp) {
            tagsDaHive = new int[]{30, 31, 32, 33};
            nomeHive = "Vermelha - Lado Oposto da Audiencia";
        }
        if (gamepad1.dpad_right && !ultimoRight) {
            tagsDaHive = new int[]{34, 35, 36, 37};
            nomeHive = "Vermelha - Lado da Audiencia";
        }
        if (gamepad1.dpad_down && !ultimoDown) {
            tagsDaHive = new int[]{38, 39, 40, 41};
            nomeHive = "Azul - Lado da Audiencia";
        }
        if (gamepad1.dpad_left && !ultimoLeft) {
            tagsDaHive = new int[]{42, 43, 44, 45};
            nomeHive = "Azul - Lado Oposto da Audiencia";
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private boolean idPertenceAHive(int id) {
        for (int idPermitido : tagsDaHive) {
            if (idPermitido == id) {
                return true;
            }
        }
        return false;
    }

    /*
     * No SDK atual, somente AprilTagSingleDetection possui ID individual.
     * AprilTagClusterDetection representa uma hive/conjunto e não possui
     * um único ID para exibir.
     */
    private int obterIdDaTag(AprilTagDetection deteccao) {
        if (deteccao instanceof AprilTagSingleDetection) {
            return ((AprilTagSingleDetection) deteccao).id;
        }
        return -1;
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

    private boolean deteccaoPertenceAHive(AprilTagDetection deteccao) {
        if (deteccao instanceof AprilTagSingleDetection) {
            return idPertenceAHive(obterIdDaTag(deteccao));
        }

        // Um cluster já representa a hive como um alvo único; não existe
        // ID individual para compará-lo com os arrays de tags.
        return deteccao instanceof AprilTagClusterDetection;
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
