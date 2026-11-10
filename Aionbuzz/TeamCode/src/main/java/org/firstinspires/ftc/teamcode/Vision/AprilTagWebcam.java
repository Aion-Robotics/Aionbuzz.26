package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;


import java.util.ArrayList;
import java.util.List;


public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // Configuração do processador de AprilTags
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(
                        DistanceUnit.CM,
                        AngleUnit.DEGREES
                )
                .build();

        // Configuração do portal de visão
        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hwMap.get(
                                WebcamName.class,
                                "Webcam 1"
                        )
                )
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagClusterDetection getClusterByName(
            String shortName
    ) {
        for (AprilTagDetection detection :
                detectedTags) {

            if (detection
                    instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null
                        && cluster.metadata.shortName
                        .equals(shortName)) {

                    return cluster;
                }
            }
        }

        return null;
    }



    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection =
                        (AprilTagSingleDetection) detection;

                if (singleDetection.id == id) {
                    return singleDetection;
                }
            }
        }

        return null;
    }

    public void stop() {

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void displayDetectionTelemetry(
            AprilTagDetection detectedId
    ) {
        if (!(detectedId instanceof AprilTagSingleDetection)) {
            return;
        }

        AprilTagSingleDetection singleDetection =
                (AprilTagSingleDetection) detectedId;

        String tagName = "Tag desconhecida";

        if (singleDetection.metadata != null) {
            tagName = singleDetection.metadata.name;
        }

        telemetry.addLine(
                String.format(
                        "\n==== (ID %d) %s",
                        singleDetection.id,
                        tagName
                )
        );

        telemetry.addLine(
                String.format(
                        "XYZ %6.1f %6.1f %6.1f (cm)",
                        singleDetection.ftcPose.x,
                        singleDetection.ftcPose.y,
                        singleDetection.ftcPose.z
                )
        );

        telemetry.addLine(
                String.format(
                        "PRY %6.1f %6.1f %6.1f (deg)",
                        singleDetection.ftcPose.pitch,
                        singleDetection.ftcPose.roll,
                        singleDetection.ftcPose.yaw
                )
        );

        telemetry.addLine(
                String.format(
                        "RBE %6.1f %6.1f %6.1f (cm, deg, deg)",
                        singleDetection.ftcPose.range,
                        singleDetection.ftcPose.bearing,
                        singleDetection.ftcPose.elevation
                )
        );
        telemetry.update();
    }

    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal == null) {
            return null;
        }

        return visionPortal.getCameraState();

    }


}
