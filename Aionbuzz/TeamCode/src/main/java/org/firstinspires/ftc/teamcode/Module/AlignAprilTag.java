package org.firstinspires.ftc.teamcode.Module;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Repository.RepositoryAprilTag;

public class AlignAprilTag extends OpMode {

    private RepositoryAprilTag controller;

    @Override
    public void init() {
        controller = new RepositoryAprilTag(hardwareMap);
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("UP = RED SCORING | RIGHT = RED AUDIENCE");
        telemetry.addLine("DOWN = BLUE AUDIENCE | LEFT = BLUE SCORING");
        telemetry.addLine("A = Manual/Alinhamento");
        telemetry.addLine("Potência Inicial: 10%");
        telemetry.update();
    }

    @Override
    public void start() {
        controller.start();
    }

    @Override
    public void loop() {
        controller.update(gamepad1, telemetry);
    }

    @Override
    public void stop() {
        if (controller != null){
            controller.stop();
        }
    }
}