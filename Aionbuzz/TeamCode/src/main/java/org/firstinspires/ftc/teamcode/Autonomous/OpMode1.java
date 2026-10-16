package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Teste OpMode Autonomous")
public class OpMode1 extends OpMode {

    private DcMotor esq;
    private DcMotor dir;

    private ElapsedTime tempo = new ElapsedTime();
    private int etapa = 0;

    @Override
    public void init() {
        esq = hardwareMap.get(DcMotor.class, "motorEsq");
        dir = hardwareMap.get(DcMotor.class, "motorDir");

        dir.setDirection(DcMotorSimple.Direction.REVERSE);

        parar();

        telemetry.addLine("Inicializado");
    }

    @Override
    public void start() {
        etapa = 1;
        tempo.reset();
    }

    @Override
    public void loop() {
        switch (etapa){
            case 1:
                esq.setPower(0.75);
                dir.setPower(0.75);

                if (tempo.seconds() >= 1.5){
                    parar();
                    etapa = 2;
                    tempo.reset();
                }
                break;

            case 2:
                stop();

                if (tempo.seconds() >= 0.2){
                    etapa = 3;
                    tempo.reset();
                }
                break;

            case 3:
                esq.setPower(-0.60);
                dir.setPower(0.60);

                if (tempo.seconds() >= 0.7){
                    parar();
                    etapa = 4;
                    tempo.reset();
                }
                break;

            case 4:
                parar();
                if (tempo.seconds() >= 0.2){
                    etapa = 5;
                    tempo.reset();
                }
                break;

            case 5:
                esq.setPower(0.75);
                dir.setPower(0.75);

                if (tempo.seconds() >= 1.5){
                    parar();
                    etapa = 6;
                    tempo.reset();
               }
                break;

            case 6:
                parar();
                telemetry.addLine("Autônomo finalizado");
                break;
        }

        telemetry.addData("Tempo", "%.2f", tempo.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        parar();
    }

    private void parar(){
        esq.setPower(0);
        dir.setPower(0);
    }
}
