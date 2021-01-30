// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DrivetrainConstants {
        public static final int flDrive = 0;
        public static final int frDrive = 1;
        public static final int blDrive = 2;
        public static final int brDrive = 3;  
        
        public static final double deadband = .1;
    }

    public final class JoystickConstants {
        //Controllers
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
            
        //XboxOne Joysticks (axes)
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;
        
        //XboxOne Buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int BUMPER_LEFT = 5;
        public static final int BUMPER_RIGHT = 6;
        public static final int LOGO_LEFT = 7;
        public static final int LOGO_RIGHT = 8;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;

    }

    public final class IntakeConstants {
        public static final int intakeMotor = 5;
    }

    public final class HopperConstants {
        public static final int hopperMotor = 6;
    }

    public final class ShooterConstants {
        public static final int shooterMotor = 7;
        public static final double defaultVelocity = 4000;
    }
}
