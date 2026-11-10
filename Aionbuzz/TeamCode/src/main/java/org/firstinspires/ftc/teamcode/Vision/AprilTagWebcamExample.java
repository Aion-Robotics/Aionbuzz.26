package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Autonomous(name = "Webcam")
public class AprilTagWebcamExample extends OpMode {

    private final AprilTagWebcam aprilTagWebcam =
            new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("VisionPortal inicializado");
        telemetry.addLine("Camera sendo inicializada...");
        telemetry.addLine(
                "Camera Stream: menu de tres pontos do Driver Hub"
        );
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clear();
        telemetry.addLine("Camera iniciada");
        telemetry.update();
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        telemetry.addData(
                "Estado da camera",
                aprilTagWebcam.getCameraState()
        );

        telemetry.addData(
                "Quantidade de detecções",
                aprilTagWebcam.getDetectedTags().size()
        );

        for (AprilTagDetection detection :
                aprilTagWebcam.getDetectedTags()) {

            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection =
                        (AprilTagSingleDetection) detection;

                telemetry.addData(
                        "ID detectado",
                        singleDetection.id
                );

                telemetry.addData(
                        "Centro",
                        "%.0f, %.0f",
                        singleDetection.center.x,
                        singleDetection.center.y
                );

                aprilTagWebcam.displayDetectionTelemetry(
                        singleDetection
                );
            }
        }

        AprilTagDetection tag30 =
                aprilTagWebcam.getTagBySpecificId(30);

        if (tag30 != null) {
            telemetry.addLine("Tag 30 encontrada");
        } else {
            telemetry.addLine("Tag 30 nao encontrada");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}

