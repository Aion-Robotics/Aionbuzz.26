package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RepositoryShooter {
    private final DcMotorEx motor;

    public static final double DEFAULT_TARGET_RPM = 5990.4;

    private static final double READY_TOLERANCE_FRACTION = 0.05;
    private static final double MIN_READY_TOLERANCE_TPS = 40.0;

    private double targetRpm = 0.0;

    public RepositoryShooter(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "motorShooter");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stop();
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm);
    }

    public void setTargetRpmFromDriver(double rpm){
        setTargetRpm(Range.clip(rpm, 0.0, DEFAULT_TARGET_RPM));
    }


    public void update() {
        double targetTicksPerSecond = rpmToTicksPerSecond(targetRpm);
        motor.setVelocity(targetTicksPerSecond);
    }


    public void stop(){
        targetRpm = 0.0;
        motor.setPower(0.0);
    }

    public double getTargetRpm(){
        return targetRpm;
    }

    public double getTargetTicksPerSecond(){
        return rpmToTicksPerSecond(targetRpm);
    }

    public double getVelocityTicksPerSecond(){
        return motor.getVelocity();
    }

    public double getVelocityRpm(){
        double ticksPerRevolution = motor.getMotorType().getTicksPerRev();
        if (ticksPerRevolution <= 0.0){
            return 0.0;
        }
        return motor.getVelocity()/ticksPerRevolution*60.0;
    }

    public boolean isAtSpeed(){
        if (targetRpm <= 0.0){
            return false;
        }
        double target = getTargetTicksPerSecond();
        double tolerance = Math.max(
                MIN_READY_TOLERANCE_TPS, Math.abs(target)*READY_TOLERANCE_FRACTION
        );

        return Math.abs(motor.getVelocity() - target) <= tolerance;
    }

    public double getTicksPerRevolution(){
        return motor.getMotorType().getTicksPerRev();
    }

    private double rpmToTicksPerSecond(double rpm){
        return rpm/60.0*getTicksPerRevolution();
    }

    public double getMotorMaxTicksPerSecond() {
        return motor.getMotorType().getMaxRPM() *
                getTicksPerRevolution() / 60.0;
    }
    public double getMotorMaxRpm() {
        return motor.getMotorType().getMaxRPM();
    }

}
