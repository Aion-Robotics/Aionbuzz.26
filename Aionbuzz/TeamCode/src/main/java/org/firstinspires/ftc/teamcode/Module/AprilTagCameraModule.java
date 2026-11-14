package org.firstinspires.ftc.teamcode.Module;

import android.util.Size;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;

public class AprilTagCameraModule {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private AprilTagProcessor processor;
    private VisionPortal portal;

    private int[] tagsDaHive = {30, 31, 32, 33};
    private String nomeHive = "Vermelha - Lado Oposto da Audiência";

    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    public void init(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder().build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(processor)
                .build();
    }

    public void selecionarHive(Gamepad gamepad){
        if (gamepad.dpad_up && !ultimoUp){
            tagsDaHive = new int[]{30, 31, 32, 33};
            nomeHive = "Vermelha - Lado Oposto da Audiência";
        }
        if (gamepad.dpad_right && !ultimoRight){
            tagsDaHive = new int[]{34, 35, 36, 37};
            nomeHive = "Vermelha - Lado da Audiência";
        }
        if (gamepad.dpad_down && !ultimoDown){
            tagsDaHive = new int[]{38, 39, 40, 41};
            nomeHive = "Azul - Lado da Audiência";
        }
        if (gamepad.dpad_left && !ultimoLeft){
            tagsDaHive = new int[]{42, 43, 44, 45};
            nomeHive = "Azul - Lado Oposto da Audiência";
        }

        ultimoUp = gamepad.dpad_up;
        ultimoRight = gamepad.dpad_right;
        ultimoDown = gamepad.dpad_down;
        ultimoLeft = gamepad.dpad_left;
    }

    public AprilTagDetection encontrarMelhorAlvo(){
        if (processor == null){
            return null;
        }

        List<AprilTagDetection> deteccoes = processor.getDetections();
        AprilTagDetection melhor = null;
        double menorRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection deteccao : deteccoes){
            if (deteccao.ftcPose == null){
                continue;
            }
            if (!idPertenceAHive(obterIdDaTag(deteccao))){
                continue;
            }
            if (deteccao.ftcPose.range < menorRange){
                menorRange = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }
        return melhor;
    }

    public void adicionarTelemetria(Telemetry telemetry){
        List<AprilTagDetection> deteccoes = processor.getDetections();

        telemetry.addData("Portal", portal.getCameraState());
        telemetry.addData("Hive", nomeHive);
        telemetry.addData("Detecções brutas", deteccoes.size());

        AprilTagDetection alvo = encontrarMelhorAlvo();

        if (alvo == null){
            telemetry.addLine("Nenhuma Tag Válida da Hive Selecionada");
        return;
        }

        telemetry.addData("Tag Alvo", obterIdDaTag(alvo));
        telemetry.addData("Range", "%.1f pol", alvo.ftcPose.range);
        telemetry.addData("Bearing", "%.1f graus", alvo.ftcPose.bearing);
        telemetry.addData("Yaw da Tag", "%.1f graus", alvo.ftcPose.yaw);
    }

    public String getNomeHive(){
        return nomeHive;
    }

    public void close(){
        if (portal != null){
            portal.close();
        }
    }

    private boolean idPertenceAHive(int id){
        for (int permitido : tagsDaHive){
            if (permitido == id){
                return true;
            }
        }
        return false;
    }

    public int obterIdDaTag(AprilTagDetection deteccao) {
        try {
            Field campoId;

            try {
                campoId = deteccao.getClass().getField("id");
            } catch (NoSuchFieldException e) {
                campoId = deteccao.getClass().getDeclaredField("id");
                campoId.setAccessible(true);
            }

            return campoId.getInt(deteccao);
        } catch (Exception ignorado) {
            try {
                Method metodoId = deteccao.getClass().getMethod("getId");
                Object resultado = metodoId.invoke(deteccao);
                return ((Number) resultado).intValue();
            } catch (Exception ignoradoTambem) {
                return -1;
            }
        }
    }
}
