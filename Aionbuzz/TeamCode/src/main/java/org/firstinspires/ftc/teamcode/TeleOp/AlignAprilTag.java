package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;

@TeleOp(name = "Align AprilTag")
public class AlignAprilTag extends OpMode {

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private DcMotor motorIntake;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean modoAlinhamento = false;
    private boolean ultimoA = false;

    private boolean ultimoUp = false;
    private boolean ultimoRight = false;
    private boolean ultimoDown = false;
    private boolean ultimoLeft = false;

    private static final double DISTANCIA_ALVO = 18.0;
    private static final double KP_GIRO = 0.018;
    private static final double KP_DISTANCIA = 0.035;
    private static final double MAX_GIRO = 0.35;
    private static final double MAX_AVANCO = 0.35;
    private static final double TOLERANCIA_GIRO = 2.0;
    private static final double TOLERANCIA_DISTANCIA = 1.5;

    private int[] tagsPermitidas = {30, 31, 32, 33};
    private String nomeGrupo = "Vermelha - lado oposto da audiencia";

    @Override
    public void init() {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        motorEsq.setDirection(DcMotor.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("BIOBUZZ AprilTag pronto");
        telemetry.addData("Grupo", nomeGrupo);
        telemetry.addLine("A = alterna alinhamento/manual");
    }

    @Override
    public void loop() {
        selecionarGrupo();

        if (gamepad1.a && !ultimoA) {
            modoAlinhamento = !modoAlinhamento;
        }
        ultimoA = gamepad1.a;

        if (modoAlinhamento) {
            alinharComAprilTag();
        } else {
            dirigirManualmente();
        }

        motorIntake.setPower(gamepad1.left_bumper ? 0.75 : 0.0);

        telemetry.addData("Modo", modoAlinhamento ? "ALINHAMENTO" : "MANUAL");
        telemetry.addData("Grupo", nomeGrupo);
        telemetry.update();
    }

    private void dirigirManualmente() {
        double frente = -gamepad1.left_stick_y;
        double giro = gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;

        motorEsq.setPower(Range.clip(
                (frente + giro) * velocidade, -1.0, 1.0));
        motorDir.setPower(Range.clip(
                (frente - giro) * velocidade, -1.0, 1.0));
    }

    private void alinharComAprilTag() {
        AprilTagDetection alvo = encontrarMelhorAlvo(
                aprilTag.getDetections());

        if (alvo == null || alvo.ftcPose == null) {
            pararBase();
            telemetry.addLine("Nenhuma tag do grupo encontrada");
            return;
        }

        double bearing = alvo.ftcPose.bearing;
        double distancia = alvo.ftcPose.range;
        double erroDistancia = distancia - DISTANCIA_ALVO;

        double giro = Range.clip(
                bearing * KP_GIRO, -MAX_GIRO, MAX_GIRO);
        double avanco = Range.clip(
                erroDistancia * KP_DISTANCIA,
                -MAX_AVANCO, MAX_AVANCO);

        boolean centralizado =
                Math.abs(bearing) <= TOLERANCIA_GIRO;
        boolean naDistancia =
                Math.abs(erroDistancia) <= TOLERANCIA_DISTANCIA;

        if (centralizado && naDistancia) {
            pararBase();
            telemetry.addLine("ALINHADO COM A HIVE");
        } else {
            motorEsq.setPower(Range.clip(
                    avanco + giro, -1.0, 1.0));
            motorDir.setPower(Range.clip(
                    avanco - giro, -1.0, 1.0));
        }

        int id = obterIdDaTag(alvo);
        telemetry.addData("Tag detectada", id);
        telemetry.addData("Bearing", "%.1f graus", bearing);
        telemetry.addData("Distancia", "%.1f pol", distancia);
        telemetry.addData("Erro distancia", "%.1f pol", erroDistancia);
    }

    private AprilTagDetection encontrarMelhorAlvo(
            List<AprilTagDetection> deteccoes) {

        AprilTagDetection melhor = null;
        double menorDistancia = Double.MAX_VALUE;

        for (AprilTagDetection deteccao : deteccoes) {
            int id = obterIdDaTag(deteccao);

            if (!tagEstaNoGrupo(id)
                    || deteccao.ftcPose == null) {
                continue;
            }

            if (deteccao.ftcPose.range < menorDistancia) {
                menorDistancia = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }

        return melhor;
    }

    /*
     * Compatibilidade entre versões/classe importada do SDK.
     * Primeiro procura um campo público chamado id.
     * Se não existir, procura um método getId().
     */
    private int obterIdDaTag(AprilTagDetection deteccao) {
        try {
            Field campoId = deteccao.getClass().getField("id");
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

    private boolean tagEstaNoGrupo(int id) {
        for (int tag : tagsPermitidas) {
            if (tag == id) {
                return true;
            }
        }
        return false;
    }

    private void selecionarGrupo() {
        if (gamepad1.dpad_up && !ultimoUp) {
            tagsPermitidas = new int[]{30, 31, 32, 33};
            nomeGrupo = "Vermelha - lado oposto da audiencia";
        }

        if (gamepad1.dpad_right && !ultimoRight) {
            tagsPermitidas = new int[]{34, 35, 36, 37};
            nomeGrupo = "Vermelha - lado da audiencia";
        }

        if (gamepad1.dpad_down && !ultimoDown) {
            tagsPermitidas = new int[]{38, 39, 40, 41};
            nomeGrupo = "Azul - lado da audiencia";
        }

        if (gamepad1.dpad_left && !ultimoLeft) {
            tagsPermitidas = new int[]{42, 43, 44, 45};
            nomeGrupo = "Azul - lado oposto da audiencia";
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private void pararBase() {
        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);
    }

    @Override
    public void stop() {
        pararBase();
        motorIntake.setPower(0.0);

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
