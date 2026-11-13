package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;

@TeleOp(name = "Align AprilTag V2")
public class AlignAprilTagV2 extends OpMode {

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private DcMotor motorIntake;
    private CRServo servoTransfer;
    private DcMotorEx motorShooter;
    private IMU imu;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean modoAlinhamento = false;
    private boolean ultimoA = false;
    private boolean ultimoUp = false;
    private boolean ultimoRight = false;
    private boolean ultimoDown = false;
    private boolean ultimoLeft = false;

    private enum FaseAlinhamento {
        APROXIMAR,
        GIRAR_COSTAS,
        PRONTO
    }

    private FaseAlinhamento faseAlinhamento = FaseAlinhamento.APROXIMAR;
    private double yawAlvoCostas = 0.0;

    private static final double DISTANCIA_ALVO = 18.0;
    private static final double KP_GIRO_TAG = 0.018;
    private static final double KP_GIRO_IMU = 0.010;
    private static final double KP_DISTANCIA = 0.035;

    private static final double MAX_GIRO_TAG = 0.20;
    private static final double MAX_GIRO_IMU = 0.25;
    private static final double MAX_AVANCO = 0.20;

    private static final double TOLERANCIA_GIRO = 1.0;
    private static final double TOLERANCIA_DISTANCIA = 1.5;
    private static final double TOLERANCIA_YAW = 2.0;

    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_POR_VOLTA_SHOOTER = 28.0;

    /*
     * Cada array representa uma hive/grupo separado.
     * Altere os IDs somente se a configuração oficial do seu campo for outra.
     */
    private int[] tagsPermitidas = {30, 31, 32, 33};
    private String nomeGrupo = "Vermelha - lado oposto da audiencia";

    @Override
    public void init() {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        servoTransfer = hardwareMap.get(CRServo.class, "servoTransfer");
        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        imu = hardwareMap.get(IMU.class, "imu");

        motorEsq.setDirection(DcMotor.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setVelocity(0.0);

        RevHubOrientationOnRobot orientacao = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientacao));
        imu.resetYaw();

        // A C920 possui calibração integrada do SDK em 640x480.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        // O nome precisa ser exatamente o nome usado no Robot Configuration.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Grupo", nomeGrupo);
        telemetry.addLine("Camera: C920 / 640x480");
        telemetry.addLine("A = alterna manual/alinhamento");
        telemetry.addLine("D-pad = seleciona a hive/grupo");
        telemetry.update();
    }

    @Override
    public void start() {
        faseAlinhamento = FaseAlinhamento.APROXIMAR;
        modoAlinhamento = false;
    }

    @Override
    public void loop() {
        selecionarGrupo();

        // Diagnóstico bruto: executado antes do filtro dos grupos.
        exibirDiagnosticoCamera();

        if (gamepad1.a && !ultimoA) {
            modoAlinhamento = !modoAlinhamento;

            if (modoAlinhamento) {
                iniciarAlinhamento();
            } else {
                faseAlinhamento = FaseAlinhamento.APROXIMAR;
                pararBase();
            }
        }
        ultimoA = gamepad1.a;

        if (modoAlinhamento) {
            alinharComAprilTag();
        } else {
            dirigirManualmente();
            exibirAprilTagDetectada();
        }

        motorIntake.setPower(gamepad1.left_bumper ? 1.0 : 0.0);
        servoTransfer.setPower(gamepad1.left_trigger > 0.1 ? -1.0 : 0.0);
        controlarShooter();

        telemetry.addData("Modo", modoAlinhamento ? "ALINHAMENTO" : "MANUAL");
        telemetry.addData("Grupo selecionado", nomeGrupo);
        telemetry.update();
    }

    private void exibirDiagnosticoCamera() {
        List<AprilTagDetection> deteccoes = aprilTag.getDetections();

        telemetry.addData("Estado da camera", visionPortal.getCameraState());
        telemetry.addData("Deteccoes brutas", deteccoes.size());

        for (int i = 0; i < deteccoes.size(); i++) {
            AprilTagDetection deteccao = deteccoes.get(i);
            int id = obterIdDaTag(deteccao);

            telemetry.addData(
                    "Tag bruta " + i,
                    "ID=%d | pose=%s | no grupo=%s",
                    id,
                    deteccao.ftcPose != null ? "VALIDA" : "NULA",
                    tagEstaNoGrupo(id) ? "SIM" : "NAO");

            if (deteccao.ftcPose != null) {
                telemetry.addData(
                        "Pose bruta " + i,
                        "bearing=%.1f | range=%.1f",
                        deteccao.ftcPose.bearing,
                        deteccao.ftcPose.range);
            }
        }
    }

    private void iniciarAlinhamento() {
        faseAlinhamento = FaseAlinhamento.APROXIMAR;
        yawAlvoCostas = 0.0;
        pararBase();
    }

    private void dirigirManualmente() {
        double frente = -gamepad1.left_stick_y;
        double giro = -gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;

        motorEsq.setPower(Range.clip((frente + giro) * velocidade, -1.0, 1.0));
        motorDir.setPower(Range.clip((frente - giro) * velocidade, -1.0, 1.0));
    }

    private void exibirAprilTagDetectada() {
        AprilTagDetection alvo = encontrarMelhorAlvo(aprilTag.getDetections());

        if (alvo == null) {
            telemetry.addLine("Nenhuma tag com pose valida no grupo selecionado");
            return;
        }

        telemetry.addData("Tag usada", obterIdDaTag(alvo));
        telemetry.addData("Bearing da tag", "%.1f graus", alvo.ftcPose.bearing);
        telemetry.addData("Distancia da tag", "%.1f pol", alvo.ftcPose.range);
    }

    private void alinharComAprilTag() {
        switch (faseAlinhamento) {
            case APROXIMAR:
                aproximarDaTag();
                break;
            case GIRAR_COSTAS:
                girarDeCostasParaTag();
                break;
            case PRONTO:
                pararBase();
                telemetry.addLine("ALINHADO - COSTAS DO ROBÔ PARA A TAG");
                break;
        }
    }

    private void aproximarDaTag() {
        AprilTagDetection alvo = encontrarMelhorAlvo(aprilTag.getDetections());

        if (alvo == null) {
            pararBase();
            telemetry.addLine("Nenhuma tag com pose valida no grupo selecionado");
            return;
        }

        double bearing = alvo.ftcPose.bearing;
        double distancia = alvo.ftcPose.range;
        double erroDistancia = distancia - DISTANCIA_ALVO;

        double giro = Range.clip(
                bearing * KP_GIRO_TAG,
                -MAX_GIRO_TAG,
                MAX_GIRO_TAG);

        double avanco = Range.clip(
                erroDistancia * KP_DISTANCIA,
                -MAX_AVANCO,
                MAX_AVANCO);

        boolean centralizado = Math.abs(bearing) <= TOLERANCIA_GIRO;
        boolean naDistancia = Math.abs(erroDistancia) <= TOLERANCIA_DISTANCIA;

        if (centralizado && naDistancia) {
            pararBase();
            yawAlvoCostas = normalizarAngulo(obterYaw() + 180.0);
            faseAlinhamento = FaseAlinhamento.GIRAR_COSTAS;
            telemetry.addLine("Tag centralizada. Girando as costas para a tag...");
            return;
        }

        motorEsq.setPower(Range.clip(avanco + giro, -1.0, 1.0));
        motorDir.setPower(Range.clip(avanco - giro, -1.0, 1.0));

        telemetry.addData("Fase", "APROXIMAR");
        telemetry.addData("Tag usada", obterIdDaTag(alvo));
        telemetry.addData("Bearing", "%.1f graus", bearing);
        telemetry.addData("Distancia", "%.1f pol", distancia);
        telemetry.addData("Erro distancia", "%.1f pol", erroDistancia);
    }

    private void girarDeCostasParaTag() {
        double yawAtual = obterYaw();
        double erroYaw = diferencaAngular(yawAlvoCostas, yawAtual);

        if (Math.abs(erroYaw) <= TOLERANCIA_YAW) {
            pararBase();
            faseAlinhamento = FaseAlinhamento.PRONTO;
            telemetry.addLine("ALINHADO - COSTAS DO ROBÔ PARA A TAG");
            return;
        }

        double giro = Range.clip(
                erroYaw * KP_GIRO_IMU,
                -MAX_GIRO_IMU,
                MAX_GIRO_IMU);

        // Se girar para o lado errado, inverta os dois sinais.
        motorEsq.setPower(giro);
        motorDir.setPower(-giro);

        telemetry.addData("Fase", "GIRAR COSTAS");
        telemetry.addData("Yaw atual", "%.1f graus", yawAtual);
        telemetry.addData("Yaw alvo", "%.1f graus", yawAlvoCostas);
        telemetry.addData("Erro yaw", "%.1f graus", erroYaw);
    }

    private void controlarShooter() {
        boolean acionado = gamepad1.right_trigger > 0.1;
        double alvoRpm = acionado ? MAX_RPM : 0.0;
        double alvoTicks = (alvoRpm * TICKS_POR_VOLTA_SHOOTER) / 60.0;

        // Sentido corrigido conforme o teste do shooter.
        motorShooter.setVelocity(alvoTicks);

        double velocidadeTicks = Math.abs(motorShooter.getVelocity());
        double velocidadeRpm = velocidadeTicks * 60.0 / TICKS_POR_VOLTA_SHOOTER;

        telemetry.addData("Shooter alvo RPM", "%.0f", alvoRpm);
        telemetry.addData("Shooter RPM real", "%.0f", velocidadeRpm);
        telemetry.addData("Shooter", acionado ? "LIGADO" : "DESLIGADO");
    }

    private double obterYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double normalizarAngulo(double angulo) {
        while (angulo > 180.0) {
            angulo -= 360.0;
        }
        while (angulo < -180.0) {
            angulo += 360.0;
        }
        return angulo;
    }

    private double diferencaAngular(double alvo, double atual) {
        return normalizarAngulo(alvo - atual);
    }

    private AprilTagDetection encontrarMelhorAlvo(
            List<AprilTagDetection> deteccoes) {

        AprilTagDetection melhor = null;
        double menorDistancia = Double.MAX_VALUE;

        for (AprilTagDetection deteccao : deteccoes) {
            if (deteccao.ftcPose == null) {
                continue;
            }

            int id = obterIdDaTag(deteccao);

            // A separação entre as hives é mantida nesta linha.
            if (!tagEstaNoGrupo(id)) {
                continue;
            }

            if (deteccao.ftcPose.range < menorDistancia) {
                menorDistancia = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }

        return melhor;
    }

    private int obterIdDaTag(AprilTagDetection deteccao) {
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
            reiniciarAlinhamentoSeNecessario();
        }

        if (gamepad1.dpad_right && !ultimoRight) {
            tagsPermitidas = new int[]{34, 35, 36, 37};
            nomeGrupo = "Vermelha - lado da audiencia";
            reiniciarAlinhamentoSeNecessario();
        }

        if (gamepad1.dpad_down && !ultimoDown) {
            tagsPermitidas = new int[]{38, 39, 40, 41};
            nomeGrupo = "Azul - lado da audiencia";
            reiniciarAlinhamentoSeNecessario();
        }

        if (gamepad1.dpad_left && !ultimoLeft) {
            tagsPermitidas = new int[]{42, 43, 44, 45};
            nomeGrupo = "Azul - lado oposto da audiencia";
            reiniciarAlinhamentoSeNecessario();
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private void reiniciarAlinhamentoSeNecessario() {
        if (modoAlinhamento) {
            faseAlinhamento = FaseAlinhamento.APROXIMAR;
            pararBase();
        }
    }

    private void pararBase() {
        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);
    }

    @Override
    public void stop() {
        pararBase();
        motorIntake.setPower(0.0);
        servoTransfer.setPower(0.0);
        motorShooter.setVelocity(0.0);

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}