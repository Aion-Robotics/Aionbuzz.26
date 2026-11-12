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

    // Processador de visão e concexão com a webcam usb
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //Estado do Programa
    private boolean modoAlinhamento = false;
    private boolean ultimoA = false;

    /*Estados anteriores do D-pad.
     Eles impedem que que um botão segurado seja processado repetidamente em todos os ciclos do loop*/
    private boolean ultimoUp = false;
    private boolean ultimoRight = false;
    private boolean ultimoDown = false;
    private boolean ultimoLeft = false;

    /*
    Parâmetros de distância do Alinhamento Automático.
    Todas as distâncias abaixo estão em polegadas porque ftcPose.range retorna a distância da câmera
    até a April Tag em Polegadas.
     */
    private static final double DISTANCIA_ALVO = 18.0;

    //Ganhos proporcionais: erro maior gera correção maior
    private static final double KP_GIRO = 0.018;
    private static final double KP_DISTANCIA = 0.035;

    //Limites de segurança para o teste automático
    private static final double MAX_GIRO = 0.20;
    private static final double MAX_AVANCO = 0.20;

    //Quando os erros ficam dentro dessa faixa, o robô para
    private static final double TOLERANCIA_GIRO = 2.0;
    private static final double TOLERANCIA_DISTANCIA = 1.5;


    /*
     * Grupo de tags aceito inicialmente. Os grupos são trocados pelo D-pad:
     * cima    = 30-33, vermelha lado oposto da audiência
     * direita = 34-37, vermelha lado da audiência
     * baixo   = 38-41, azul lado da audiência
     * esquerda= 42-45, azul lado oposto da audiência
     */
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
        //Atualiza o grupo de April Tags se algum botão do D-pad foi pressionado
        selecionarGrupo();

        //Alterna o modo somente na transição de solto para pressionado
        if (gamepad1.a && !ultimoA) {
            modoAlinhamento = !modoAlinhamento;
        }
        ultimoA = gamepad1.a;

        if (modoAlinhamento) {
            alinharComAprilTag();
        } else {
            dirigirManualmente();
        }

        motorIntake.setPower(gamepad1.left_bumper ? 1.0 : 0.0);

        telemetry.addData("Modo", modoAlinhamento ? "ALINHAMENTO" : "MANUAL");
        telemetry.addData("Grupo", nomeGrupo);
        telemetry.update();
    }

    private void dirigirManualmente() {
        double frente = -gamepad1.left_stick_y;
        double giro = -gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;

        motorEsq.setPower(Range.clip(
                (frente + giro) * velocidade, -1.0, 1.0));
        motorDir.setPower(Range.clip(
                (frente - giro) * velocidade, -1.0, 1.0));
    }

    /*
     * Procura a tag selecionada e transforma os erros de visão em potência.
     * bearing indica o erro horizontal em graus; range indica a distância.
     */
    private void alinharComAprilTag() {
        AprilTagDetection alvo = encontrarMelhorAlvo(
                aprilTag.getDetections());

        //Sem tag váida ou sem pose, o robô para por segurança
        if (alvo == null || alvo.ftcPose == null) {
            pararBase();
            telemetry.addLine("Nenhuma tag do grupo encontrada");
            return;
        }

        double bearing = alvo.ftcPose.bearing;
        double distancia = alvo.ftcPose.range;
        double erroDistancia = distancia - DISTANCIA_ALVO;

        //Controle proporcional com limites para evitar movimentos bruscos
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
            //Como a base é tank, o giro é obtido com potências diferentes
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

    /*
     * Escolhe a tag válida mais próxima dentro do grupo selecionado.
     * Tags de outros grupos são ignoradas para evitar escolher outra Hive.
     */
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
     * Lê o identificador sem depender de uma única versão da classe do SDK.
     * Primeiro tenta o campo público id; depois tenta getId().
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

    // Retorna true somente se o ID pertence ao grupo escolhido no D-pad.
    private boolean tagEstaNoGrupo(int id) {
        for (int tag : tagsPermitidas) {
            if (tag == id) {
                return true;
            }
        }
        return false;
    }

    /* Seleciona um dos quatro lados das duas Hives pelo D-pad. */
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

        // Fecha a webcam e libera os recursos de visão ao terminar o OpMode.
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
