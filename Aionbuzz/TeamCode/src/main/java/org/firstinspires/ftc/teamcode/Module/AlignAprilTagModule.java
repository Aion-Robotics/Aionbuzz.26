package org.firstinspires.ftc.teamcode.Module;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Module.AprilTagCameraModule;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AlignAprilTagModule extends OpMode {

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private IMU imu;
    private AprilTagCameraModule camera;

    private boolean alinhamento = false;
    private boolean ultimoA = false;
    private Fase fase = Fase.APROXIMAR;
    private double yawAlvoCostas;

    private static final double DISTANCIA_ALVO = 18.0;
    private static final double KP_BEARING = 0.018;
    private static final double KP_RANGE = 0.035;
    private static final double KP_YAW = 0.010;
    private static final double MAX_GIRO = 0.20;
    private static final double MAX_AVANCO = 0.20;
    private static final double MAX_GIRO_IMU = 0.25;
    private static final double TOLERANCIA_BEARING = 1.0;
    private static final double TOLERANCIA_RANGE = 1.5;
    private static final double TOLERANCIA_YAW = 2.0;

    private enum Fase {
        APROXIMAR,
        GIRAR_COSTAS,
        PRONTO
    }

    @Override
    public void init() {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        imu = hardwareMap.get(IMU.class, "imu");

        motorEsq.setDirection(DcMotor.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot orientacao =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientacao));
        imu.resetYaw();

        // A câmera é inicializada pelo módulo reutilizável.
        camera = new AprilTagCameraModule();
        camera.init(hardwareMap);

        telemetry.addLine("Align AprilTag Modular inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        imu.resetYaw();
        alinhamento = false;
        fase = Fase.APROXIMAR;
    }

    @Override
    public void loop() {
        camera.selecionarHive(gamepad1);

        if (gamepad1.a && !ultimoA) {
            alinhamento = !alinhamento;
            fase = Fase.APROXIMAR;
            pararBase();
        }
        ultimoA = gamepad1.a;

        if (alinhamento) {
            executarAlinhamento();
        } else {
            dirigirManualmente();
        }

        telemetry.addData("Modo", alinhamento ? "ALINHAMENTO" : "MANUAL");
        telemetry.addData("Fase", fase);
        camera.adicionarTelemetria(telemetry);
        telemetry.update();
    }

    private void executarAlinhamento() {
        if (fase == Fase.APROXIMAR) {
            aproximar();
        } else if (fase == Fase.GIRAR_COSTAS) {
            girarCostas();
        } else {
            pararBase();
            telemetry.addLine("PRONTO - costas voltadas para a hive");
        }
    }

    private void aproximar() {
        AprilTagDetection alvo = camera.encontrarMelhorAlvo();

        if (alvo == null) {
            pararBase();
            return;
        }

        double bearing = alvo.ftcPose.bearing;
        double erroRange = alvo.ftcPose.range - DISTANCIA_ALVO;

        double giro = limitar(
                bearing * KP_BEARING, -MAX_GIRO, MAX_GIRO);
        double avanco = limitar(
                erroRange * KP_RANGE, -MAX_AVANCO, MAX_AVANCO);

        if (Math.abs(bearing) <= TOLERANCIA_BEARING
                && Math.abs(erroRange) <= TOLERANCIA_RANGE) {
            pararBase();
            yawAlvoCostas = normalizar(obterYaw() + 180.0);
            fase = Fase.GIRAR_COSTAS;
            return;
        }

        motorEsq.setPower(limitar(
                avanco + giro, -1.0, 1.0));
        motorDir.setPower(limitar(
                avanco - giro, -1.0, 1.0));

        telemetry.addData("Bearing", "%.1f graus", bearing);
        telemetry.addData("Erro range", "%.1f pol", erroRange);
    }

    private void girarCostas() {
        double erroYaw = diferencaAngular(yawAlvoCostas, obterYaw());

        if (Math.abs(erroYaw) <= TOLERANCIA_YAW) {
            pararBase();
            fase = Fase.PRONTO;
            return;
        }

        double giro = limitar(
                erroYaw * KP_YAW, -MAX_GIRO_IMU, MAX_GIRO_IMU);

        motorEsq.setPower(giro);
        motorDir.setPower(-giro);
    }

    private void dirigirManualmente() {
        double frente = -gamepad1.left_stick_y;
        double giro = -gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;

        motorEsq.setPower(limitar(
                (frente + giro) * velocidade, -1.0, 1.0));
        motorDir.setPower(limitar(
                (frente - giro) * velocidade, -1.0, 1.0));
    }

    private double obterYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double normalizar(double angulo) {
        while (angulo > 180.0) angulo -= 360.0;
        while (angulo < -180.0) angulo += 360.0;
        return angulo;
    }

    private double diferencaAngular(double alvo, double atual) {
        return normalizar(alvo - atual);
    }

    private double limitar(double valor, double minimo, double maximo) {
        return Math.max(minimo, Math.min(maximo, valor));
    }

    private void pararBase() {
        motorDir.setPower(0.0);
        motorEsq.setPower(0.0);
    }

    @Override
    public void stop() {
        pararBase();
        if (camera != null) {
            camera.close();
        }
    }
}
