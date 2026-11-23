package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Repository.RepositoryAprilTag;

@TeleOp
public class AlignAprilTag extends OpMode {

    private RepositoryAprilTag aprilTag;

    @Override
    public void init() {
        aprilTag = new RepositoryAprilTag(hardwareMap);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("AlignAprilTag iniciado");
        telemetry.addLine("A = alterna Manual/Alinhamento");
        telemetry.update();
    }

    @Override
    public void start() {
        aprilTag.start();
    }

    @Override
    public void loop() {
        aprilTag.update(gamepad1, telemetry);

        adicionarTelemetriaAprilTag();

        atualizarShooter();
        atualizarIntake();

        telemetry.update();
    }

    private void adicionarTelemetriaAprilTag() {
        RepositoryAprilTag.DebugSnapshot debug =
                aprilTag.getDebugSnapshot();

        telemetry.addData(
                "Modo",
                aprilTag.isModoAlinhamento()
                        ? "AUTOMÁTICO"
                        : "MANUAL"
        );

        telemetry.addData("Hive", aprilTag.getHiveSelecionada());
        telemetry.addData("Alvo encontrado", debug.alvoEncontrado);
        telemetry.addData("Status", debug.status);
        telemetry.addData("Bearing", "%.2f graus", debug.bearing);
        telemetry.addData("Erro angular", "%.2f graus", debug.erroBearing);
        telemetry.addData("Range", "%.2f in", debug.range);
        telemetry.addData("Erro distância", "%.2f in", debug.erroRange);
        telemetry.addData("Potência avanço", "%.3f", debug.potenciaAvanco);
        telemetry.addData("Potência giro", "%.3f", debug.potenciaGiro);
        telemetry.addData("Ciclos estáveis", debug.ciclosEstaveis);
        telemetry.addData("Ciclos sem alvo", debug.ciclosSemAlvo);
        telemetry.addData(
                "Tempo automático",
                "%.2f s",
                debug.tempoAlinhamento
        );
    }

    private void atualizarShooter() {
        // Adicionar aqui o controle do shooter.
    }

    private void atualizarIntake() {
        // Adicionar aqui o controle do intake.
    }

    @Override
    public void stop() {
        if (aprilTag != null) {
            aprilTag.stop();
        }
    }
}