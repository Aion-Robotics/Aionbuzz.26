package org.firstinspires.ftc.teamcode.Repository;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class RepositoryAprilTag {
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final double TIMEOUT_ALINHAMENTO_SEGUNDOS = 8.0;
    private static final int MAX_CICLOS_SEM_ALVO = 10;

    private static final String META_HIVE_UP = "RED SCORING";
    private static final String META_HIVE_RIGHT = "RED AUDIENCE";
    private static final String META_HIVE_DOWN = "BLUE AUDIENCE";
    private static final String META_HIVE_LEFT = "BLUE SCORING";

    public enum Hive { RED_SCORING, RED_AUDIENCE, BLUE_AUDIENCE, BLUE_SCORING }

    public static class DebugSnapshot {
        public final boolean alvoEncontrado;
        public final double bearing, range, erroBearing, erroRange;
        public final double potenciaAvanco, potenciaGiro;
        public final int ciclosEstaveis, ciclosSemAlvo;
        public final double tempoAlinhamento;
        public final AprilTagAlignmentMath.Status status;

        public DebugSnapshot(boolean alvoEncontrado, double bearing, double range,
                             double erroBearing, double erroRange,
                             double potenciaAvanco, double potenciaGiro,
                             int ciclosEstaveis, int ciclosSemAlvo,
                             double tempoAlinhamento,
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

    private final DcMotor motorDir;
    private final DcMotor motorEsq;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final AprilTagAlignmentMath alinhamento;
    private final ElapsedTime cronometroAlinhamento = new ElapsedTime();

    private Hive hiveSelecionada = Hive.RED_SCORING;
    private boolean alinhamentoAtivo;
    private int ciclosSemAlvo;
    private String motivoSaida = "";
    private DebugSnapshot debugSnapshot;
    private double ultimaCorrecaoHeading;

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
        debugSnapshot = snapshotSemAlvo();
    }

    public void start() {
        pararBase();
        alinhamentoAtivo = false;
        alinhamento.reset();
        ciclosSemAlvo = 0;
        ultimaCorrecaoHeading = 0.0;
        motivoSaida = "";
        debugSnapshot = snapshotSemAlvo();
    }

    public void setHive(Hive hive) {
        if (hive == null) throw new IllegalArgumentException("Hive nao pode ser nula");
        hiveSelecionada = hive;
        if (alinhamentoAtivo) startAlignment();
    }

    public Hive getHiveSelecionada() { return hiveSelecionada; }

    public void startAlignment() {
        alinhamentoAtivo = true;
        alinhamento.reset();
        ciclosSemAlvo = 0;
        ultimaCorrecaoHeading = 0.0;
        motivoSaida = "";
        cronometroAlinhamento.reset();
        pararBase();
    }

    public void cancelAlignment() {
        alinhamentoAtivo = false;
        alinhamento.reset();
        ultimaCorrecaoHeading = 0.0;
        pararBase();
    }

    public boolean isAlignmentActive() { return alinhamentoAtivo; }

    public void updateAlignment(Telemetry telemetry, double deltaTime) {
        updateAlignment(telemetry, deltaTime, 0.0);
    }

    public void updateAlignment(Telemetry telemetry, double deltaTime,
                                double correcaoHeading) {
        ultimaCorrecaoHeading = Range.clip(correcaoHeading, -0.20, 0.20);
        if (!alinhamentoAtivo) return;

        if (cronometroAlinhamento.seconds() >= TIMEOUT_ALINHAMENTO_SEGUNDOS) {
            abortar("Timeout total do alinhamento");
            telemetry.addData("Alinhamento", "CANCELADO");
            telemetry.addData("Motivo", motivoSaida);
            return;
        }

        AprilTagDetection alvo = encontrarMelhorAlvo();
        if (alvo == null || alvo.ftcPose == null) {
            ciclosSemAlvo++;
            alinhamento.update(AprilTagAlignmentMath.Input.semAlvo());
            pararBase();
            debugSnapshot = snapshotSemAlvo();
            if (ciclosSemAlvo >= MAX_CICLOS_SEM_ALVO) {
                abortar("AprilTag perdida por tempo excessivo");
                telemetry.addData("Alinhamento", "CANCELADO");
                telemetry.addData("Motivo", motivoSaida);
                return;
            }
            telemetry.addData("Alinhamento", "AGUARDANDO ALVO");
            telemetry.addData("Ciclos sem alvo", "%d/%d", ciclosSemAlvo, MAX_CICLOS_SEM_ALVO);
            return;
        }

        ciclosSemAlvo = 0;
        AprilTagAlignmentMath.Output resultado = alinhamento.update(
                AprilTagAlignmentMath.Input.alvo(alvo.ftcPose.bearing, alvo.ftcPose.range),
                deltaTime);

        aplicarBase(resultado.avanco, resultado.giro + ultimaCorrecaoHeading);
        debugSnapshot = new DebugSnapshot(
                true, alvo.ftcPose.bearing, alvo.ftcPose.range,
                resultado.erroBearing, resultado.erroRange,
                resultado.avanco, resultado.giro + ultimaCorrecaoHeading,
                resultado.ciclosEstaveis, ciclosSemAlvo,
                cronometroAlinhamento.seconds(), resultado.status);

        telemetry.addData("Alinhamento", resultado.status);
        telemetry.addData("Bearing", "%.2f graus", alvo.ftcPose.bearing);
        telemetry.addData("Erro bearing", "%.2f graus", resultado.erroBearing);
        telemetry.addData("Range", "%.2f in", alvo.ftcPose.range);
        telemetry.addData("Erro range", "%.2f in", resultado.erroRange);
        telemetry.addData("Correcao heading", "%.3f", ultimaCorrecaoHeading);
    }

    public double getLastHeadingCorrection() { return ultimaCorrecaoHeading; }

    public void setDrivePower(double avanco, double giro) {
        if (!alinhamentoAtivo) aplicarBase(avanco, giro);
    }

    public DebugSnapshot getDebugSnapshot() { return debugSnapshot; }
    public String getMotivoSaida() { return motivoSaida; }

    public void stop() {
        pararBase();
        visionPortal.close();
    }

    private void abortar(String motivo) {
        alinhamentoAtivo = false;
        alinhamento.reset();
        pararBase();
        motivoSaida = motivo;
    }

    private AprilTagDetection encontrarMelhorAlvo() {
        List<AprilTagDetection> deteccoes = aprilTagProcessor.getDetections();
        AprilTagDetection melhor = null;
        double menorRange = Double.POSITIVE_INFINITY;
        for (AprilTagDetection deteccao : deteccoes) {
            if (deteccao.ftcPose == null || !(deteccao instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) deteccao;
            if (cluster.metadata == null || !cluster.metadata.name.equals(metadataDaHive())
                    || Math.abs(deteccao.ftcPose.roll) >= 90.0) continue;
            if (deteccao.ftcPose.range < menorRange) {
                menorRange = deteccao.ftcPose.range;
                melhor = deteccao;
            }
        }
        return melhor;
    }

    private DebugSnapshot snapshotSemAlvo() {
        return new DebugSnapshot(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                alinhamento.getCiclosEstaveis(), ciclosSemAlvo,
                cronometroAlinhamento.seconds(),
                AprilTagAlignmentMath.Status.PROCURANDO_ALVO);
    }

    private String metadataDaHive() {
        switch (hiveSelecionada) {
            case RED_SCORING: return META_HIVE_UP;
            case RED_AUDIENCE: return META_HIVE_RIGHT;
            case BLUE_AUDIENCE: return META_HIVE_DOWN;
            case BLUE_SCORING: return META_HIVE_LEFT;
            default: return "DESCONHECIDA";
        }
    }

    private void aplicarBase(double avanco, double giro) {
        motorEsq.setPower(Range.clip(avanco + giro, -1.0, 1.0));
        motorDir.setPower(Range.clip(avanco - giro, -1.0, 1.0));
    }

    private void pararBase() {
        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);
    }
}
