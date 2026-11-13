package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;

@TeleOp
public class AprilTagCamera extends OpMode {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;


    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private int[] tagsDaHive = {30, 31, 32, 33};
    private String nomeHive = "Vermelha - Lado Oposto da Audiência";

    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    @Override
    public void init() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Câmera Inicializada");
        telemetry.update();
    }

    @Override
    public void loop() {
        selecionarHive();

        ArrayList<AprilTagDetection> deteccoes = aprilTagProcessor.getDetections();

        AprilTagDetection melhorAlvo = encontrarMelhorAlvo(deteccoes);

        telemetry.addData("Portal", visionPortal.getCameraState());
        telemetry.addData("Resolução", "%d x %d", CAMERA_WIDTH, CAMERA_HEIGHT);
        telemetry.addData("Hive Selecionada", nomeHive);
        telemetry.addData("Detecções Recebidas", deteccoes.size());
        telemetry.update();

        if (deteccoes.isEmpty()){
            telemetry.addLine("Nenhuma Detecção Recebida pelo Processador");
        } else {
            for (int i = 0; i < deteccoes.size(); i++){
                AprilTagDetection deteccao = deteccoes.get(i);
                int id  = obterIdDaTag(deteccao);
                boolean poseValida = deteccao.ftcPose != null;

                telemetry.addData(
                        "Tag %d",
                        "ID = %d | pose = %d",
                        i + 1,
                        id,
                        poseValida ? "VALIDA" : "NULA");

                if (poseValida){
                    telemetry.addData(
                            "pose %d",
                            "range = %.1f in | bearing = &.1fº | yaw = %.1fº",
                            i + 1,
                            deteccao.ftcPose.range,
                            deteccao.ftcPose.bearing,
                            deteccao.ftcPose.yaw);
                }
            }
        }

        if (melhorAlvo == null){
            telemetry.addLine("Alvo da Hive: nenhum com Pose Válida");
        } else {
            telemetry.addData("Alvo da Hive", "ID %d", obterIdDaTag(melhorAlvo));
            telemetry.addData("Alvo range", "%.1f in", melhorAlvo.ftcPose.range);
            telemetry.addData("Alvo bearing", "%.1fº", melhorAlvo.ftcPose.bearing);
            telemetry.addLine("Câmera e pose prontas para integração posterior");
        }
        telemetry.update();
    }


    private void selecionarHive(){
        if (gamepad1.dpad_up && !ultimoUp){
            tagsDaHive = new int[]{30, 31, 32, 33};
            nomeHive = "Vermelha - Lado Oposto da Audiência";
        }
        if (gamepad1.dpad_right && !ultimoRight){
            tagsDaHive = new int[]{34, 35, 36, 37};
            nomeHive = "Vermelha - Lado da Audiência";
        }
        if (gamepad1.dpad_down && !ultimoDown){
            tagsDaHive = new int[]{38, 39, 40, 41};
            nomeHive = "Azul = Lado da Audiência";
        }
        if (gamepad1.dpad_left && !ultimoLeft){
            tagsDaHive = new int[]{42, 43, 44, 45};
            nomeHive = "Azul - Lado Oposto da Audiência";
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private AprilTagDetection encontrarMelhorAlvo(
            ArrayList<AprilTagDetection> deteccoes) {

        AprilTagDetection melhor = null;
        double menorRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection deteccao : deteccoes) {
            if (deteccao.ftcPose == null) {
                continue;
            }

            int id = obterIdDaTag(deteccao);

            if (!idPertenceAHive(id)) {
                continue;
            }

            if (deteccao.ftcPose.range < menorRange) {
                menorRange = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }

        return melhor;
    }


    private boolean idPertenceAHive(int id){
        for (int idPermitido : tagsDaHive){
            if (idPermitido == id){
                return true;
            }
        }

        return false;
    }

    private int obterIdDaTag(AprilTagDetection deteccao){
        try {
            Field campoId;
            try {
                campoId = deteccao.getClass().getField("id");
            } catch (NoSuchFieldException e){
                campoId = deteccao.getClass().getDeclaredField("id");
                campoId.setAccessible(true);
            }
            return campoId.getInt(deteccao);
        } catch (Exception ignorado){
            try {
                Method metodoId = deteccao.getClass().getMethod("getId");
                Object resultado = metodoId.invoke(deteccao);
                return ((Number) resultado).intValue();
            } catch (Exception ignoradoTambem){
                return -1;
            }
        }
    }

    @Override
    public void stop() {
        if (visionPortal != null){
            visionPortal.close();
        }
    }
}
