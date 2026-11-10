package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Disabled
public class FollowAprilTag extends OpMode {

    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotor motorEsq;
    private DcMotor motorDir;


    private static final double DESIRED_DISTANCE_CM = 50.0;
    /*
    Distância desejada entre o Robô e a Tag
    Unidade em CM
     */
    private static final double KP_DISTANCIA = 0.015;
    /*
    Ganho proporcial de distãncia
    Passível de Aumento ou diminuição dependendo da velocidade do robô
     */
    private static final double KP_ANGULO = 0.012;
    /*
    Ganho proporcional do ângulo
    Começo baixo para evitar oscilações
     */

    private static final double POTENCIA_MAXIMA_MOTOR = 0.35;
    private static final double POTENCIA_MAXIMA_GIRO = 0.25;
    // Limites de potência para primeiro teste

    private static final double TOLERANCIA_DISTANCIA_CM = 3.0;
    private static final double TOLERANCIA_ANGULO_GRAUS = 2.0;
    //Tolerâncias para considerar o robô alinhado e na distância correta


    @Override
    public void init() {
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");

        motorDir.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pararMotores();

        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("Follow April Tag 30");
        telemetry.update();
    }

    @Override
    public void start() {
        pararMotores();

        telemetry.clear();
        telemetry.addLine("Procurando Tag 30 ...");
        telemetry.update();
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        AprilTagWebcam tag30 = aprilTagWebcam.getTagBySpecificId(30);

        if (tag30 == null) {
            pararMotores();

            telemetry.addLine("Tag 30 não encontrada");
            telemetry.addData("Tags detectadas", aprilTagWebcam.getDetectedTags().size());
            telemetry.update();

            return;
        }

        if (!(tag30 instanceof AprilTagSingleDetection)) {
            pararMotores();

            telemetry.addLine("Detecção não é uma tag individual");
            telemetry.update();

            return;
        }

        AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) tag30;

        if (singleDetection.metadata == null) {
            pararMotores();

            telemetry.addLine("Tag identificada, porém sem metadata");
            telemetry.update();

            return;
        }

        double distanciaAtualCm = singleDetection.ftcPose.range;
        double anguloAtualGraus = singleDetection.ftcPose.bearing;


        double erroDistancia = distanciaAtualCm - DESIRED_DISTANCE_CM;
        double erroAngulo = anguloAtualGraus;

        double potenciaFrente =  KP_DISTANCIA * erroDistancia;
        double potenciaGiro = KP_ANGULO * erroAngulo;

        potenciaFrente = Range.clip(potenciaFrente, -POTENCIA_MAXIMA_MOTOR, POTENCIA_MAXIMA_MOTOR);
        potenciaGiro = Range.clip(potenciaGiro, -POTENCIA_MAXIMA_GIRO, POTENCIA_MAXIMA_GIRO);

        if (Math.abs(erroDistancia) <= TOLERANCIA_DISTANCIA_CM){
            potenciaFrente = 0.0;
        }

        if (Math.abs(erroAngulo) <= TOLERANCIA_ANGULO_GRAUS){
            potenciaGiro = 0.0;
        }

        double potenciaEsquerda = potenciaFrente - potenciaGiro;
        double potenciaDireita = potenciaFrente + potenciaGiro;

        double maiorPotencia = Math.max(Math.abs(potenciaEsquerda), Math.abs(potenciaDireita));

        if (maiorPotencia > 1.0) {
            potenciaEsquerda /= maiorPotencia;
            potenciaDireita /= maiorPotencia;
        }

        boolean objetivoAlcançado =
                Math.abs(erroDistancia)
                        <= TOLERANCIA_DISTANCIA_CM
                        && Math.abs(erroAngulo)
                        <= TOLERANCIA_ANGULO_GRAUS;

        if (objetivoAlcançado) {
            potenciaEsquerda = 0.0;
            potenciaDireita = 0.0;
        }
        definirPotencia(potenciaEsquerda, potenciaDireita);

        telemetry.addData("Tag", singleDetection.id);
        telemetry.addData("Distância Atual (cm)", "%.1f", distanciaAtualCm);
        telemetry.addData("Ângulo Bearing (graus)", "%.1f", erroAngulo);
        telemetry.addData("Potência Esquerda", "%.2f", potenciaEsquerda);
        telemetry.addData("Potência Direita", "%.2f", potenciaDireita);
        telemetry.addData("Objetivo", objetivoAlcançado ? "Alcançado" : "A Caminho");
        telemetry.update();

    }

    private void pararMotores() {
        definirPotencia(0.0, 0.0);
    }
    private void definirPotencia(
            double esquerda,
            double direita
    ) {
        motorEsq.setPower(esquerda);
        motorDir.setPower(direita);
    }

    @Override
    public void stop() {
        pararMotores();
        aprilTagWebcam.stop();;
    }
}
