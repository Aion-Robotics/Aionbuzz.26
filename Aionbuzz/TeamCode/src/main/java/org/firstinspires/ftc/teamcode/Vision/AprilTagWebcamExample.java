package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Webcam")
public class AprilTagWebcamExample extends OpMode {

    private final AprilTagWebcam aprilTagWebcam =
            new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("VisionPortal inicializado");
        telemetry.addLine("Camera sendo inicializada...");
        telemetry.update();
    }

    @Override
    public void start() {
        /*
         * O VisionPortal normalmente já começa a transmitir
         * automaticamente. Este método só será necessário se
         * você tiver parado o streaming anteriormente.
         */
        aprilTagWebcam.resumeStreaming();

        telemetry.clear();
        telemetry.addLine("Camera iniciada");
        telemetry.update();
    }

    @Override
    public void loop() {
        /*
         * O FTC chama loop() repetidamente.
         * Portanto, a lista de detecções é atualizada continuamente.
         */
        aprilTagWebcam.update();

        telemetry.addData(
                "Estado da camera",
                aprilTagWebcam.getCameraState()
        );

        telemetry.addData(
                "Tags detectadas",
                aprilTagWebcam.getDetectedTags().size()
        );

        /*
         * Mostra todas as tags detectadas.
         * Isso facilita o teste, pois você não precisa
         * saber antecipadamente qual é o ID da tag.
         */
        for (AprilTagDetection detection :
                aprilTagWebcam.getDetectedTags()) {

            aprilTagWebcam.displayDetectionTelemetry(detection);
        }

        /*
         * Procura especificamente a tag de ID 30.
         */
        AprilTagDetection tag30 =
                aprilTagWebcam.getTagBySpecificId(30);

        if (tag30 != null) {
            telemetry.addLine("Tag 30 encontrada");
        } else {
            telemetry.addLine("Tag 30 nao encontrada");
        }

        /*
         * Envia todo o conteúdo acumulado para o Driver Hub.
         */
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
