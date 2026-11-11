package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@Autonomous
public class AprilTagsPracticed extends OpMode {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    protected final double POLEGADA_EM_CM = 2.54;

    @Override
    public void init() {
        initVision();
    }

    @Override
    public void loop() {
        updateTelemetry();
    }

    private void initVision(){
        // Registra a Webcam no sistema
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(webcam, aprilTag);
    }
    private void updateTelemetry(){
        List<AprilTagDetection>  detectedTags = aprilTag.getDetections();
        telemetry.addData("AprilTags Detected", detectedTags.size());

        for (AprilTagDetection detection : detectedTags){
            if (detection instanceof AprilTagSingleDetection){
                AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                if (singleDetection.metadata != null) {
                    // Informações da Leitura
                    telemetry.addData("ID", singleDetection.id);
                    telemetry.addData("Metadata", singleDetection.metadata.name);

                    // Posicionamento das Tags
                    telemetry.addData("\nX (Distancia Lateral)", "%.2f cm",(singleDetection.ftcPose.x)*POLEGADA_EM_CM);
                    telemetry.addData("Y (Distancia Frontal)","%.2f cm",(singleDetection.ftcPose.y)*POLEGADA_EM_CM);
                    telemetry.addData("Z (Altura)","%.2f cm",(singleDetection.ftcPose.z)*POLEGADA_EM_CM);
                }
                else
                    telemetry.addData("ID %d", "Undefined", singleDetection.id);
            }
            else{
                AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;

                telemetry.addData("==== Tag Cluster ====", "%s\n", clusterDetection.metadata.name);

                telemetry.addData("Percentual Tags", "%d", clusterDetection.percentClusterFound);
                telemetry.addData("\nX (Distancia Lateral)", "%.2f",(clusterDetection.ftcPose.x)*POLEGADA_EM_CM);
                telemetry.addData("Y (Distancia Frontal)","%.2f cm",(clusterDetection.ftcPose.y)*POLEGADA_EM_CM);
                telemetry.addData("Z (Altura)","%.2f cm",(clusterDetection.ftcPose.z)*POLEGADA_EM_CM);
            }
        }
    }

}
