package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Autonomous
public class Cluster extends OpMode {

    private final AprilTagWebcam aprilTagWebcam =
            new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(
                hardwareMap,
                telemetry
        );
    }

    @Override
    public void start() {
        telemetry.clear();
        telemetry.update();
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        telemetry.addData(
                "Total de deteccoes",
                aprilTagWebcam
                        .getDetectedTags()
                        .size()
        );

        for (AprilTagDetection detection :
                aprilTagWebcam.getDetectedTags()) {

            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                telemetry.addLine(
                        "TIPO: CLUSTER BIOBUZZ"
                );

                telemetry.addData(
                        "Nome do cluster",
                        cluster.metadata.name
                );

                telemetry.addData(
                        "Nome curto",
                        cluster.metadata.shortName
                );

                telemetry.addData(
                        "Porcentagem encontrada",
                        cluster.percentClusterFound
                );

                telemetry.addData(
                        "Range (cm)",
                        "%.1f",
                        cluster.ftcPose.range
                );

                telemetry.addData(
                        "Bearing (graus)",
                        "%.1f",
                        cluster.ftcPose.bearing
                );

                telemetry.addData(
                        "Yaw (graus)",
                        "%.1f",
                        cluster.ftcPose.yaw
                );
            }

            else if (detection
                    instanceof AprilTagSingleDetection) {

                AprilTagSingleDetection single =
                        (AprilTagSingleDetection) detection;

                telemetry.addLine(
                        "TIPO: TAG INDIVIDUAL"
                );

                telemetry.addData(
                        "ID",
                        single.id
                );

                telemetry.addData(
                        "Metadata",
                        single.metadata != null
                                ? single.metadata.name
                                : "NULA"
                );
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
