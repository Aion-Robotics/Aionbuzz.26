package org.firstinspires.ftc.teamcode.Module;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;

@TeleOp
public class AlignAprilTag extends OpMode {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final String META_HIVE_UP = "RED SCORING";
    private static final String META_HIVE_RIGHT = "RED AUDIENCE";
    private static final String META_HIVE_DOWN = "BLUE AUDIENCE";
    private static final String META_HIVE_LEFT = "BLUE SCORING";

    private static final double DISTANCIA_ALVO = 18.0;
    private static final double KP_BEARING = 0.015;
    private static final double KP_RANGE = 0.025;
    private static final double KP_YAW = 0.018;

    private static final double TOLERANCIA_BEARING = 2.0;
    private static final double TOLERANCIA_RANGE = 2.0;
    private static final double TOLERANCIA_YAW = 3.0;

    private static final double MAX_GIRO = 0.15;
    private static final double MAX_AVANCO = 0.10;
    private static final double MAX_GIRO_IMU = 0.20;

    private static final int CICLOS_ALVO_ESTAVEL = 3;
    private static final boolean INVERTER_GIRO_IMU = false;

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private enum Hive {
        RED_SCORING,
        RED_AUDIENCE,
        BLUE_AUDIENCE,
        BLUE_SCORING
    }

    private enum Fase {
        APROXIMAR,
        GIRAR_COSTAS,
        PRONTO
    }

    private Hive hiveSelecionada = Hive.RED_SCORING;
    private Fase fase = Fase.APROXIMAR;

    private boolean modoAlinhamento;
    private boolean ultimoA;
    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    private int ciclosAlvoValido;
    private double yawAlvoCostas;

    @Override
    public void init() {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        imu = hardwareMap.get(IMU.class, "imu");

        motorEsq.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot orientacao = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientacao));
        imu.resetYaw();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.setMsTransmissionInterval(100);
        mostrarInstrucoes();
    }

    @Override
    public void start() {
        pararBase();
        imu.resetYaw();
        modoAlinhamento = false;
        fase = Fase.APROXIMAR;
        ciclosAlvoValido = 0;
    }

    @Override
    public void loop() {
        selecionarHive();
        alternarModo();

        if (modoAlinhamento){
            alinharComAprilTag();
        }else {
            dirigirManualmente();
            telemetry.addData("Modo", "MANUAL");
            telemetry.addData("Hive", metadataDaHive());
        }
        telemetry.update();
    }

    private void selecionarHive(){
        if (gamepad1.dpad_up && !ultimoUp){
            hiveSelecionada = Hive.RED_SCORING;
            reiniciarAlinhamentoSeNecessario();
        }
        if (gamepad1.dpad_right && !ultimoRight){
            hiveSelecionada = Hive.RED_AUDIENCE;
            reiniciarAlinhamentoSeNecessario();
        }
        if (gamepad1.dpad_down && !ultimoDown){
            hiveSelecionada = Hive.BLUE_AUDIENCE;
            reiniciarAlinhamentoSeNecessario();
        }
        if (gamepad1.dpad_left && !ultimoLeft){
            hiveSelecionada = Hive.BLUE_SCORING;
            reiniciarAlinhamentoSeNecessario();
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private void alternarModo(){
        if (gamepad1.a && !ultimoA){
            modoAlinhamento = !modoAlinhamento;
            if (modoAlinhamento){
                iniciarAlinhamento();
            }else {
                fase = Fase.APROXIMAR;
                ciclosAlvoValido = 0;
                pararBase();
            }
        }
        ultimoA = gamepad1.a;
    }

    private void iniciarAlinhamento(){
        fase = Fase.APROXIMAR;
        ciclosAlvoValido = 0;
        yawAlvoCostas = 0.0;
        pararBase();
    }

    private void alinharComAprilTag(){
        AprilTagDetection alvo = encontrarMelhorAlvo();

        if (fase == Fase.APROXIMAR){
            aproximarDaHive(alvo);
        } else if (fase == Fase.GIRAR_COSTAS) {
            girarDeCostas();
        } else {
            pararBase();
            telemetry.addData("Modo", "ALINHAMENTO");
            telemetry.addData("Fase", "PRONTO");
            telemetry.addData("Hive", metadataDaHive());
        }
    }

    private void aproximarDaHive(AprilTagDetection alvo){
        if (alvo == null || alvo.ftcPose == null){
            pararBase();
            ciclosAlvoValido = 0;
            telemetry.addData("Modo", "ALINHAMENTO");
            telemetry.addData("Fase", "APROXIMAR");
            telemetry.addData("Hive", metadataDaHive());
            telemetry.addLine("Nenhum Cluster Válido da Hive Selecionada");
            return;
        }

        double bearing = alvo.ftcPose.bearing;
        double range = alvo.ftcPose.range;
        double erroRange = range - DISTANCIA_ALVO;

        boolean bearingCorreto = Math.abs(bearing) <= TOLERANCIA_BEARING;
        boolean distanciaCorreta = Math.abs(erroRange) <= TOLERANCIA_RANGE;

        if (bearingCorreto && distanciaCorreta){
            pararBase();
            ciclosAlvoValido++;
            if (ciclosAlvoValido >= CICLOS_ALVO_ESTAVEL){
                yawAlvoCostas = normalizarAngulo(obterYaw() + 180.0);
                fase = Fase.GIRAR_COSTAS;
                ciclosAlvoValido = 0;
            }

            exibirDadosDoAlvo(alvo, bearing, range, erroRange);
            telemetry.addData("Fase", "CENTRALIZADO - Confirmado");
            telemetry.addData("Ciclos Estáveis", ciclosAlvoValido);
            return;
        }

        ciclosAlvoValido = 0;
        double giro = Range.clip(bearing * KP_BEARING, -MAX_GIRO, MAX_GIRO);
        double avanco = erroRange * KP_RANGE;

        if (Math.abs(erroRange) > TOLERANCIA_RANGE
                && Math.abs(avanco) < 0.08) {
            avanco = Math.copySign(0.08, avanco);
        }

        avanco = Range.clip(avanco, -MAX_AVANCO, MAX_AVANCO);

        aplicarBase(avanco, giro);

        exibirDadosDoAlvo(alvo, bearing, range, erroRange);
        telemetry.addData("Fase", "APROXIMAR");
        telemetry.addData("Potência avanço", "%.2f", avanco);
        telemetry.addData("Potência giro", "%.2f", giro);
    }

    private void girarDeCostas(){
        double yawAtual = obterYaw();
        double erroYaw = diferencaAngular(yawAlvoCostas, yawAtual);

        if (Math.abs(erroYaw) <= TOLERANCIA_YAW){
            pararBase();
            fase = Fase.PRONTO;
            return;
        }

        double giro = Range.clip(erroYaw * KP_YAW, -MAX_GIRO_IMU, MAX_GIRO_IMU);
        if (INVERTER_GIRO_IMU){
            giro = -giro;
        }

        motorEsq.setPower(giro);
        motorDir.setPower(-giro);

        telemetry.addData("Modo", "ALINHAMENTO");
        telemetry.addData("Fase", "GIRAR COSTAS");
        telemetry.addData("Hive", metadataDaHive());
        telemetry.addData("Yaw atual", "%.1f", yawAtual);
        telemetry.addData("Erro Yaw", "%.1f", erroYaw);
        telemetry.addData("Potência giro IMU", "%.2f", giro);
    }

    private AprilTagDetection encontrarMelhorAlvo(){
        ArrayList<AprilTagDetection> deteccoes = aprilTagProcessor.getDetections();
        AprilTagDetection melhor = null;
        double menorRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection deteccao : deteccoes){
            if (deteccao.ftcPose == null){
                continue;
            }
            if (!deteccaoPertenceAHive(deteccao)){
                continue;
            }
            if (deteccao.ftcPose.range < menorRange){
                menorRange = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }
        return melhor;
    }

    private boolean deteccaoPertenceAHive(AprilTagDetection deteccao) {
        if (!(deteccao instanceof AprilTagClusterDetection)) {
            return false;
        }
        if (deteccao.ftcPose == null) {
            return false;
        }
        AprilTagClusterDetection cluster =
                (AprilTagClusterDetection) deteccao;
        if (cluster.metadata == null) {
            return false;
        }
        if (!cluster.metadata.name.equals(metadataDaHive())) {
            return false;
        }
        return Math.abs(deteccao.ftcPose.roll) < 90.0;
    }


    private void exibirDadosDoAlvo(AprilTagDetection alvo, double bearing, double range, double erroRange){
        telemetry.addData("Modo", "Alinhamento");
        telemetry.addData("Hive", metadataDaHive());
        telemetry.addData("Alvo", identificacao(alvo));
        telemetry.addData("Bearing", "%.1f graus", bearing);
        telemetry.addData("Range", "%.1f in", range);
        telemetry.addData("Erro Range", "%.1f in", erroRange);
    }

    private void dirigirManualmente(){
        double frente = -gamepad1.left_stick_y;
        double giro = -gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;

        aplicarBase(frente * velocidade, giro * velocidade);
    }

    private void aplicarBase(double avanco, double giro){
        double potenciaEsq = Range.clip(avanco+giro, -1.0, 1.0);
        double potenciaDir = Range.clip(avanco-giro, -1.0, 1.0);

        motorEsq.setPower(potenciaEsq);
        motorDir.setPower(potenciaDir);
    }

    private String metadataDaHive(){
        switch (hiveSelecionada){
            case RED_SCORING:
                return META_HIVE_UP;
            case RED_AUDIENCE:
                return META_HIVE_RIGHT;
            case BLUE_AUDIENCE:
                return META_HIVE_DOWN;
            case BLUE_SCORING:
                return META_HIVE_LEFT;
            default:
                return "DESCONHECIDA";
        }
    }

    private String identificacao(AprilTagDetection deteccao){
        if (deteccao instanceof AprilTagClusterDetection){
            return "CLUSTER/HIVE";
        }
        if (deteccao instanceof AprilTagSingleDetection){
            return "ID = " + ((AprilTagSingleDetection)deteccao).id;
        }
        return "DESCONHECIDO";
    }

    private void reiniciarAlinhamentoSeNecessario(){
        if (modoAlinhamento){
            iniciarAlinhamento();
        }
    }

    private double obterYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double normalizarAngulo(double angulo){
        while (angulo > 180.0){
            angulo -= 360.0;
        }
        while (angulo < -180.0){
            angulo += 360.0;
        }
        return angulo;
    }

    private double diferencaAngular(double alvo, double atual){
        return normalizarAngulo(alvo - atual);
    }

    private void pararBase(){
        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);
    }

    private void mostrarInstrucoes(){
        telemetry.addLine("UP = RED SCORING | RIGHT = RED AUDIENCE");
        telemetry.addLine(" DOWN = BLUE AUDIENCE | LEFT = BLUE SCORING");
        telemetry.addLine("A = Manual/Alinhamento");
        telemetry.addLine("Potência Inicial: 10%");
        telemetry.update();
    }

    @Override
    public void stop() {
        pararBase();
        if (visionPortal != null){
            visionPortal.close();
        }
    }
}