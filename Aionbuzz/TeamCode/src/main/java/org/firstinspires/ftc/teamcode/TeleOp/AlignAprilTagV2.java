package org.firstinspires.ftc.teamcode.TeleOp;

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

@TeleOp(name = "Align AprilTag 2")
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

    // Referência de direção usada pelo IMU enquanto o robô se aproxima da tag.
    private double yawReferenciaAlinhamento = 0.0;
    private boolean referenciaYawInicializada = false;

    private static final double DISTANCIA_ALVO = 18.0;
    private static final double KP_GIRO_TAG = 0.018;
    private static final double KP_CORRECAO_IMU = 0.010;
    private static final double KP_DISTANCIA = 0.035;
    private static final double MAX_GIRO_TAG = 0.20;
    private static final double MAX_CORRECAO_IMU = 0.10;
    private static final double MAX_AVANCO = 0.20;
    private static final double TOLERANCIA_GIRO = 1.0;
    private static final double TOLERANCIA_DISTANCIA = 1.5;
    private static final double TOLERANCIA_YAW = 2.0;

    // Configuração do shooter.
    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_POR_VOLTA_SHOOTER = 28.0;

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

        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setVelocity(0.0);

        // Orientação escolhida: Logo à direita e USB para cima.
        RevHubOrientationOnRobot orientacao = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientacao));
        imu.resetYaw();

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("BIOBUZZ AlignAprilTag integrado pronto");
        telemetry.addData("Grupo", nomeGrupo);
        telemetry.addLine("A = alterna manual/alinhamento");
        telemetry.addLine("D-pad = seleciona grupo de AprilTags");
        telemetry.addLine("Gatilho direito = shooter no maximo; solte para parar");
        telemetry.update();
    }

    @Override
    public void start() {
        referenciaYawInicializada = false;
    }

    @Override
    public void loop() {
        selecionarGrupo();

        if (gamepad1.a && !ultimoA) {
            modoAlinhamento = !modoAlinhamento;
            if (modoAlinhamento) {
                yawReferenciaAlinhamento = obterYaw();
                referenciaYawInicializada = true;
            } else {
                referenciaYawInicializada = false;
            }
        }
        ultimoA = gamepad1.a;

        if (modoAlinhamento) {
            alinharComAprilTag();
        } else {
            dirigirManualmente();
        }

        motorIntake.setPower(gamepad1.left_bumper ? 1.0 : 0.0);
        servoTransfer.setPower(gamepad1.left_trigger > 0.1 ? -1.0 : 0.0);
        controlarShooter();

        telemetry.addData("Modo", modoAlinhamento ? "ALINHAMENTO" : "MANUAL");
        telemetry.addData("Grupo", nomeGrupo);
        telemetry.update();
    }

    private void dirigirManualmente() {
        double frente = -gamepad1.left_stick_y;
        double giro = -gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;

        motorEsq.setPower(Range.clip((frente + giro) * velocidade, -1.0, 1.0));
        motorDir.setPower(Range.clip((frente - giro) * velocidade, -1.0, 1.0));
    }

    private void alinharComAprilTag() {
        if (!referenciaYawInicializada) {
            yawReferenciaAlinhamento = obterYaw();
            referenciaYawInicializada = true;
        }

        AprilTagDetection alvo = encontrarMelhorAlvo(aprilTag.getDetections());
        if (alvo == null || alvo.ftcPose == null) {
            pararBase();
            telemetry.addLine("Nenhuma tag do grupo encontrada");
            return;
        }

        double bearing = alvo.ftcPose.bearing;
        double distancia = alvo.ftcPose.range;
        double erroDistancia = distancia - DISTANCIA_ALVO;

        // O bearing aponta para a tag; o IMU mantém a direção de referência
        // e reduz oscilações quando a câmera perde pequenas leituras.
        double erroYaw = diferencaAngular(yawReferenciaAlinhamento, obterYaw());
        double correcaoTag = Range.clip(bearing * KP_GIRO_TAG,
                -MAX_GIRO_TAG, MAX_GIRO_TAG);
        double correcaoImu = Range.clip(erroYaw * KP_CORRECAO_IMU,
                -MAX_CORRECAO_IMU, MAX_CORRECAO_IMU);
        double giro = Range.clip(correcaoTag + correcaoImu, -MAX_GIRO_TAG,
                MAX_GIRO_TAG);
        double avanco = Range.clip(erroDistancia * KP_DISTANCIA,
                -MAX_AVANCO, MAX_AVANCO);

        boolean centralizado = Math.abs(bearing) <= TOLERANCIA_GIRO;
        boolean naDistancia = Math.abs(erroDistancia) <= TOLERANCIA_DISTANCIA;
        boolean direcaoEstavel = Math.abs(erroYaw) <= TOLERANCIA_YAW;

        if (centralizado && naDistancia && direcaoEstavel) {
            pararBase();
            telemetry.addLine("ALINHADO COM A HIVE");
        } else {
            motorEsq.setPower(Range.clip(avanco + giro, -1.0, 1.0));
            motorDir.setPower(Range.clip(avanco - giro, -1.0, 1.0));
        }

        telemetry.addData("Tag detectada", obterIdDaTag(alvo));
        telemetry.addData("Bearing", "%.1f graus", bearing);
        telemetry.addData("Yaw IMU", "%.1f graus", obterYaw());
        telemetry.addData("Erro yaw", "%.1f graus", erroYaw);
        telemetry.addData("Distancia", "%.1f pol", distancia);
        telemetry.addData("Erro distancia", "%.1f pol", erroDistancia);
    }

    private void controlarShooter() {
        // O shooter só é acionado pelo gatilho direito, sempre no máximo.
        boolean acionado = gamepad1.right_trigger > 0.1;
        double alvoRpm = acionado ? MAX_RPM : 0.0;
        double alvoTicks = (alvoRpm * TICKS_POR_VOLTA_SHOOTER) / 60.0;

        motorShooter.setVelocity(-alvoTicks);
        double velocidadeTicks = Math.abs(motorShooter.getVelocity());
        double velocidadeRpm = velocidadeTicks * 60.0 / TICKS_POR_VOLTA_SHOOTER;

        telemetry.addData("Shooter alvo RPM", "%.0f", alvoRpm);
        telemetry.addData("Shooter RPM real", "%.0f", velocidadeRpm);
        telemetry.addData("Shooter", acionado ? "LIGADO - MAXIMO" : "DESLIGADO");
    }

    private double obterYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double diferencaAngular(double alvo, double atual) {
        double diferenca = alvo - atual;
        while (diferenca > 180.0) {
            diferenca -= 360.0;
        }
        while (diferenca < -180.0) {
            diferenca += 360.0;
        }
        return diferenca;
    }

    private AprilTagDetection encontrarMelhorAlvo(
            List<AprilTagDetection> deteccoes) {
        AprilTagDetection melhor = null;
        double menorDistancia = Double.MAX_VALUE;

        for (AprilTagDetection deteccao : deteccoes) {
            int id = obterIdDaTag(deteccao);
            if (!tagEstaNoGrupo(id) || deteccao.ftcPose == null) {
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
        servoTransfer.setPower(0.0);
        motorShooter.setVelocity(0.0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}