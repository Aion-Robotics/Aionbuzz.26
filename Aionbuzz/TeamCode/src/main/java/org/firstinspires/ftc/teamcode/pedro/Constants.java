package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static final String MOTOR_ESQ = "motorEsq";
    public static final String MOTOR_DIR = "motorDir";
    public static final String IMU = "imu";

    public static final double WHEEL_DIAMETER_INCHES = 3.779;

    public static final double TICKS_PER_WHEEL_REVOLUTION =
            537.6 * 19.2;

    public static final double TRACK_WIDTH_INCHES = 14.5;

    public static final double TICKS_TO_INCHES =
            (Math.PI * WHEEL_DIAMETER_INCHES)
                    / TICKS_PER_WHEEL_REVOLUTION;

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new DifferentialLocalizer(
                        hardwareMap,
                        MOTOR_ESQ,
                        MOTOR_DIR,
                        IMU,
                        TICKS_TO_INCHES,
                        TRACK_WIDTH_INCHES
                ),

                new DifferentialDriveTrain(
                        hardwareMap,
                        MOTOR_ESQ,
                        MOTOR_DIR
                ),

                null
        );
    }
}
