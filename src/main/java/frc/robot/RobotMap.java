/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static final int rangefinderPort = 1;
    // public static final int rangefinderModule = 1;

    public class DRIVETRAIN {
        public static final int LEFT_MASTER_CHANNEL = 13;
        public static final int LEFT_FOLLOWER_ONE_CHANNEL = 14;
        public static final int LEFT_FOLLOWER_TWO_CHANNEL = 15;
        public static final int RIGHT_MASTER_CHANNEL = 17;
        public static final int RIGHT_FOLLOWER_ONE_CHANNEL = 1;
        public static final int RIGHT_FOLLOWER_TWO_CHANNEL = 2;

        public static final int CURRENT_LIMIT = 50;

        public static final double GEAR_RATIO = 7.58 / 12 / 15;
        public static final double WHEEL_RADIUS_FEET = .25;
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS_FEET;
    }
    //gear ratio is 7.58:12:15

    public class DRIVE {
        public static final double PITCH_THRESHOLD_DEGREES = 5;
        public static final double PITCH_ALARM_THRESHOLD = 10;

        public static final double MIN_CORRECTION_FACTOR = 0;
        public static final double MAX_CORRECTION_FACTOR = 1;
        public static final double CORRECTION_FACTOR = (MAX_CORRECTION_FACTOR - MIN_CORRECTION_FACTOR) / (45 - PITCH_THRESHOLD_DEGREES);

        public static final double PID_PITCH_CORRECTION_FACTOR = 1;
        public static final double PID_YAW_CORRECTION_FACTOR = 1;

        public static final double KP = .1;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double DRIVE_STRAIGHT_POWER = .3;
    }

    public class CLIMBER {
        public static final int FRONT_FOOT_FORWARD = 0;
        public static final int FRONT_FOOT_REVERSE = 4;

        public static final int REAR_FEET_FORWARD = 5;
        public static final int REAR_FEET_REVERSE = 4;

        public static final int WINCH_LEFT_MASTER_CHANNEL = 6;
        public static final int WINCH_LEFT_FOLLOWER_CHANNEL = 8;
        public static final int WINCH_RIGHT_MASTER_CHANNEL = 5;
        public static final int WINCH_RIGHT_FOLLOWER_CHANNEL = 9;

        public static final double WINCH_POWER = .8;

        public static final int CURRENT_LIMIT = 30; // amps
        public static final int OPTICAL_SENSOR = 2;
    }

    public class CARGO_ARM {
        public static final int ARM_MOTOR_CHANNEL = 7;

        public static final int POTENTIOMETER = 3;

        public static final int DISK_BRAKE_FORWARD = 3;
        public static final int DISK_BRAKE_REVERSE = 2;
    }

    public static class SPADE {
        public static final int FORWARD_CHANNEL = 2;
        public static final int REVERSE_CHANNEL = 3;

        public static final int EJECTORS = 1;
    }

    public static class CARGO_MANIPULATOR {
        public static final int ROLLER = 4;
    }

    public class OI {
        public static final int LEFT_JOY = 0;
        public static final int RIGHT_JOY = 1;
        public static final int XBOX = 2;
        public static final int BUTTON_BOARD = 3;
        // xbox
        public static final int EJECT_HATCH = 6; // right bumper
        // button board
        public static final int SECOND_LEVEL_CLIMBER = 11;
        public static final int SECOND_LEVEL_UNREADY = 8;
        public static final int REAR_FEET = 14;
        public static final int ALL_LVL_2 = 5;

        public static final int WINCH_CLIMBER = 4;
        public static final int WINCH_REVERSE = 10;
        public static final int AUTOMATIC_PARK = 9;

        public static final int TOGGLE_ANTI_TIP = 13;
    }

    public static final int PRESSURE_SENSOR_PORT = 0;

    public static final int PCM_A = 0;
    public static final int PCM_B = 1;
}
