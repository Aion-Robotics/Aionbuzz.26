package org.firstinspires.ftc.teamcode.Repository;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class RepositoryAprilTag {
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final String META_HIVE_UP = "RED SCORING";
    private static final String META_HIVE_RIGHT = "RED AUDIENCE";
    private static final String META_HIVE_DOWN = "BLUE AUDIENCE";
    private static final String META_HIVE_LEFT = "BLUE SCORING";

    private final DcMotor motorDir;
    private final DcMotor motorEsq;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final AprilTagAlignmentMath alinhamento;
    private final ElapsedTime cronometroAlinhamento = new ElapsedTime();
    private final ElapsedTime cronometroLoop = new ElapsedTime();

    // Valores devem ser ajustados conforme a taxa de atualização da câmera.
    private static final double TIMEOUT_ALINHAMENTO_SEGUNDOS = 8.0;
    private static final int MAX_CICLOS_SEM_ALVO = 10;

    private int ciclosSemAlvo;
    private String motivoSaidaAutomatica = "";
    private DebugSnapshot debugSnapshot;

    private Hive hiveSelecionada = Hive.RED_SCORING;
    private boolean modoAlinhamento;
    private boolean ultimoA;
    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    public enum Hive {
        RED_SCORING,
        RED_AUDIENCE,
        BLUE_AUDIENCE,
        BLUE_SCORING
    }

    public static class DebugSnapshot {
        public final boolean alvoEncontrado;
        public final double bearing;
        public final double range;
        public final double erroBearing;
        public final double erroRange;
        public final double potenciaAvanco;
        public final double potenciaGiro;
        public final int ciclosEstaveis;
        public final int ciclosSemAlvo;
        public final double tempoAlinhamento;
        public final AprilTagAlignmentMath.Status status;

        public DebugSnapshot(boolean alvoEncontrado, double bearing,
                             double range, double erroBearing,
                             double erroRange, double potenciaAvanco,
                             double potenciaGiro, int ciclosEstaveis,
                             int ciclosSemAlvo, double tempoAlinhamento,
                             AprilTagAlignmentMath.Status status) {
            this.alvoEncontrado = alvoEncontrado;
            this.bearing = bearing;
            this.range = range;
            this.erroBearing = erroBearing;
            this.erroRange = erroRange;
            this.potenciaAvanco = potenciaAvanco;
            this.potenciaGiro = potenciaGiro;
            this.ciclosEstaveis = ciclosEstaveis;
            this.ciclosSemAlvo = ciclosSemAlvo;
            this.tempoAlinhamento = tempoAlinhamento;
            this.status = status;
        }
    }

    public RepositoryAprilTag(HardwareMap hardwareMap) {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");

        motorEsq.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        alinhamento = new AprilTagAlignmentMath();
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void start() {
        pararBase();
        modoAlinhamento = false;
        alinhamento.reset();
        ciclosSemAlvo = 0;
        motivoSaidaAutomatica = "";
        cronometroLoop.reset();
        debugSnapshot = criarSnapshotSemAlvo();
        ultimoA = false;
        ultimoUp = false;
        ultimoRight = false;
        ultimoDown = false;
        ultimoLeft = false;
    }

    public void update(Gamepad gamepad, Telemetry telemetry) {
        double deltaTime = cronometroLoop.seconds();
        cronometroLoop.reset();

        selecionarHive(gamepad);
        alternarModo(gamepad);

        if (modoAlinhamento) {
            alinharComAprilTag(telemetry, deltaTime);
        } else {
            dirigirManualmente(gamepad);
            telemetry.addData("Modo", "MANUAL");
            telemetry.addData("Hive", metadataDaHive());
            debugSnapshot = criarSnapshotSemAlvo();
        }
    }

    public void stop() {
        pararBase();
        visionPortal.close();
    }

    public Hive getHiveSelecionada() {
        return hiveSelecionada;
    }

    public boolean isModoAlinhamento() {
        return modoAlinhamento;
    }

    public DebugSnapshot getDebugSnapshot() {
        return debugSnapshot;
    }

    private void selecionarHive(Gamepad gamepad) {
        if (gamepad.dpad_up && !ultimoUp) {
            hiveSelecionada = Hive.RED_SCORING;
            reiniciarAlinhamentoSeNecessario();
        }
        if (gamepad.dpad_right && !ultimoRight) {
            hiveSelecionada = Hive.RED_AUDIENCE;
            reiniciarAlinhamentoSeNecessario();
        }
        if (gamepad.dpad_down && !ultimoDown) {
            hiveSelecionada = Hive.BLUE_AUDIENCE;
            reiniciarAlinhamentoSeNecessario();
        }
        if (gamepad.dpad_left && !ultimoLeft) {
            hiveSelecionada = Hive.BLUE_SCORING;
            reiniciarAlinhamentoSeNecessario();
        }

        ultimoUp = gamepad.dpad_up;
        ultimoRight = gamepad.dpad_right;
        ultimoDown = gamepad.dpad_down;
        ultimoLeft = gamepad.dpad_left;
    }

    private void alternarModo(Gamepad gamepad) {
        if (gamepad.a && !ultimoA) {
            modoAlinhamento = !modoAlinhamento;
            if (modoAlinhamento) {
                iniciarAlinhamento();
            } else {
                alinhamento.reset();
                pararBase();
            }
        }
        ultimoA = gamepad.a;
    }

    private void iniciarAlinhamento() {
        alinhamento.reset();
        ciclosSemAlvo = 0;
        motivoSaidaAutomatica = "";
        cronometroAlinhamento.reset();
        pararBase();
    }

    private void abortarAlinhamento(String motivo) {
        modoAlinhamento = false;
        alinhamento.reset();
        ciclosSemAlvo = 0;
        motivoSaidaAutomatica = motivo;
        pararBase();
    }

    private void alinharComAprilTag(Telemetry telemetry, double deltaTime) {
        if (cronometroAlinhamento.seconds() >= TIMEOUT_ALINHAMENTO_SEGUNDOS) {
            abortarAlinhamento("Timeout total do alinhamento");
            mostrarFalhaAutomatica(telemetry);
            return;
        }

        AprilTagDetection alvo = encontrarMelhorAlvo();
        AprilTagAlignmentMath.Output resultado;

        if (alvo == null || alvo.ftcPose == null) {
            ciclosSemAlvo++;
            resultado = alinhamento.update(AprilTagAlignmentMath.Input.semAlvo());
            pararBase(); // Nunca manter a última potência sem uma leitura válida.
            debugSnapshot = criarSnapshotSemAlvo();

            if (ciclosSemAlvo >= MAX_CICLOS_SEM_ALVO) {
                abortarAlinhamento("AprilTag perdida por tempo excessivo");
                mostrarFalhaAutomatica(telemetry);
                return;
            }

            mostrarStatus(resultado, telemetry);
            telemetry.addLine("Alvo perdido; aguardando nova detecção...");
            telemetry.addData("Ciclos sem alvo", "%d/%d",
                    ciclosSemAlvo, MAX_CICLOS_SEM_ALVO);
            return;
        }

        ciclosSemAlvo = 0;
        resultado = alinhamento.update(AprilTagAlignmentMath.Input.alvo(
                alvo.ftcPose.bearing,
                alvo.ftcPose.range), deltaTime);

        debugSnapshot = new DebugSnapshot(
                true,
                alvo.ftcPose.bearing,
                alvo.ftcPose.range,
                resultado.erroBearing,
                resultado.erroRange,
                resultado.avanco,
                resultado.giro,
                resultado.ciclosEstaveis,
                ciclosSemAlvo,
                cronometroAlinhamento.seconds(),
                resultado.status
        );

        aplicarBase(resultado.avanco, resultado.giro);
        exibirDadosDoAlvo(alvo, resultado, telemetry);
        telemetry.addData("Tempo automático", "%.1f/%.1f s",
                cronometroAlinhamento.seconds(), TIMEOUT_ALINHAMENTO_SEGUNDOS);
    }

    private DebugSnapshot criarSnapshotSemAlvo() {
        return new DebugSnapshot(
                false,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                alinhamento.getCiclosEstaveis(),
                ciclosSemAlvo,
                cronometroAlinhamento.seconds(),
                AprilTagAlignmentMath.Status.PROCURANDO_ALVO
        );
    }

    private void mostrarFalhaAutomatica(Telemetry telemetry) {
        telemetry.addData("Modo", "MANUAL - ALINHAMENTO CANCELADO");
        telemetry.addData("Motivo", motivoSaidaAutomatica);
        telemetry.addLine("Aperte A novamente para tentar de novo.");
    }

    private void mostrarStatus(AprilTagAlignmentMath.Output resultado, Telemetry telemetry) {
        telemetry.addData("Modo", "ALINHAMENTO");
        telemetry.addData("Fase", nomeStatus(resultado.status));
        telemetry.addData("Hive", metadataDaHive());
        telemetry.addData("Ciclos Estáveis", resultado.ciclosEstaveis);
    }

    private void exibirDadosDoAlvo(AprilTagDetection alvo,
                                   AprilTagAlignmentMath.Output resultado,
                                   Telemetry telemetry) {
        mostrarStatus(resultado, telemetry);
        telemetry.addData("Alvo", identificacao(alvo));
        telemetry.addData("Bearing", "%.1f graus", alvo.ftcPose.bearing);
        telemetry.addData("Range", "%.1f in", alvo.ftcPose.range);
        telemetry.addData("Erro Range", "%.1f in", resultado.erroRange);
        telemetry.addData("Potência avanço", "%.2f", resultado.avanco);
        telemetry.addData("Potência giro", "%.2f", resultado.giro);
    }

    private String nomeStatus(AprilTagAlignmentMath.Status status) {
        switch (status) {
            case PROCURANDO_ALVO:
                return "PROCURANDO ALVO";
            case APROXIMANDO:
                return "APROXIMAR";
            case CENTRALIZADO:
                return "CENTRALIZADO - CONFIRMANDO";
            case PRONTO:
                return "PRONTO";
            default:
                return "DESCONHECIDO";
        }
    }

    private AprilTagDetection encontrarMelhorAlvo() {
        List<AprilTagDetection> deteccoes = aprilTagProcessor.getDetections();
        AprilTagDetection melhor = null;
        double menorRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection deteccao : deteccoes) {
            if (deteccao.ftcPose == null || !deteccaoPertenceAHive(deteccao)) {
                continue;
            }
            if (deteccao.ftcPose.range < menorRange) {
                menorRange = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }
        return melhor;
    }

    private boolean deteccaoPertenceAHive(AprilTagDetection deteccao) {
        if (!(deteccao instanceof AprilTagClusterDetection) || deteccao.ftcPose == null) {
            return false;
        }

        AprilTagClusterDetection cluster = (AprilTagClusterDetection) deteccao;
        return cluster.metadata != null
                && cluster.metadata.name.equals(metadataDaHive())
                && Math.abs(deteccao.ftcPose.roll) < 90.0;
    }

    private void dirigirManualmente(Gamepad gamepad) {
        double frente = -gamepad.left_stick_y;
        double giro = -gamepad.left_stick_x;
        double velocidade = gamepad.right_bumper ? 1.0 : 0.5;
        aplicarBase(frente * velocidade, giro * velocidade);
    }

    private void aplicarBase(double avanco, double giro) {
        double potenciaEsq = Range.clip(avanco + giro, -1.0, 1.0);
        double potenciaDir = Range.clip(avanco - giro, -1.0, 1.0);
        motorEsq.setPower(potenciaEsq);
        motorDir.setPower(potenciaDir);
    }

    private String metadataDaHive() {
        switch (hiveSelecionada) {
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

    private String identificacao(AprilTagDetection deteccao) {
        if (deteccao instanceof AprilTagClusterDetection) {
            return "CLUSTER/HIVE";
        }
        if (deteccao instanceof AprilTagSingleDetection) {
            return "ID = " + ((AprilTagSingleDetection) deteccao).id;
        }
        return "DESCONHECIDO";
    }

    private void reiniciarAlinhamentoSeNecessario() {
        if (modoAlinhamento) {
            iniciarAlinhamento();
        }
    }

    private void pararBase() {
        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);
    }

}