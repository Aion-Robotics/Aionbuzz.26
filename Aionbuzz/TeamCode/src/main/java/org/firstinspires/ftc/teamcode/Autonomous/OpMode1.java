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
        telemetry.update();

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
                esq.setPower(0.50);
                dir.setPower(0.50);

                if (tempo.seconds() >= 3.5){
                    parar();
                    etapa = 2;
                    tempo.reset();
                }
                break;

            case 2:
                parar();

                if (tempo.seconds() >= 0.2){
                    etapa = 3;
                    tempo.reset();
                }
                break;

            case 3:
                esq.setPower(-0.60);
                dir.setPower(0.60);

                if (tempo.seconds() >= 0.35){
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
                esq.setPower(0.50);
                dir.setPower(0.50);

                if (tempo.seconds() >= 2.75){
                    parar();
                    etapa = 6;
                    tempo.reset();
               }
                break;

            case 6:
                parar();
                if (tempo.seconds() >= 0.2){
                    etapa = 7;
                    tempo.reset();
                }
                break;


            case 7:
                esq.setPower(-0.60);
                dir.setPower(0.60);

                if (tempo.seconds() >= 0.35){
                    parar();
                    etapa = 8;
                    tempo.reset();
                }
                break;

            case 8:
                parar();
                if (tempo.seconds() >= 0.2){
                    etapa = 9;
                    tempo.reset();
                }
                break;

            case 9:
                esq.setPower(0.50);
                dir.setPower(0.50);

                if (tempo.seconds() >= 3.25){
                    parar();
                    etapa = 10;
                    tempo.reset();
                }
                break;

            case 10:
                esq.setPower(-0.60);
                dir.setPower(0.60);

                if (tempo.seconds() >= 0.35){
                    parar();
                    etapa = 11;
                    tempo.reset();
                }
                break;

            case 11:
                parar();
                if (tempo.seconds() >= 0.2){
                    etapa = 12;
                    tempo.reset();
                }
                break;

            case 12:
                esq.setPower(0.50);
                dir.setPower(0.50);

                if (tempo.seconds() >= 3.25) {
                    parar();
                    etapa = 13;
                    tempo.reset();
                }
                break;

            case 13:
                parar();
                break;



        }


        telemetry.addData("Etapa", etapa);
        telemetry.addData("Tempo da etapa", "%.2f", tempo.seconds());
        telemetry.addData("Motor esquerdo", "%.2f", esq.getPower());
        telemetry.addData("Motor direito", "%.2f", dir.getPower());
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
