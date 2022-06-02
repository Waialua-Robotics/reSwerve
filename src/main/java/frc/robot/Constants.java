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

    public final class ID {
        public static final int FLdrive = 11;
        public static final int FLpivot = 21;
        public static final int FLencoder = 1;
        
        public static final int FRdrive = 12;
        public static final int FRpivot = 22;
        public static final int FRencoder = 2;
        
        public static final int RLdrive = 13;
        public static final int RLpivot = 23;
        public static final int RLencoder = 3;

        public static final int RRdrive = 14;
        public static final int RRpivot = 24;
        public static final int RRencoder = 4;

        public static final int driver = 0;
        public static final int operator = 1;
    }   // this should house id of devices

    public final class OI {
        public final class driver {
            public static final int Xaxis = 0;
            public static final int Yaxis = 1;
            public static final int ROTaxis = 2;
            public static final int FieldCentric = 3;

            public static final double Deadband = 0.05;
        }   // this should house ports for driver
        public final class operator {

        }   // this should house ports for operator
    }   // this should house operator inpunt (controllers)

    public final class ModuleConstants {
        public static final double DRIVE_GEAR_RATIO = 7;
        public static final double PIVOT_GEAR_RATIO = 46;
        public static final double ENCODER_TO_VELOCITY = 40*Math.PI/8062976;
        public static final double POSITION_TO_ANGLE = (double) 360/2048;    
        public static final double ENCODER_OFFSET = 270;   // degrees
    }   // module constants 

    public final class DriveConstants {
        public static final double TRACK_WIDTH = 0.57785;   // meters
        public static final double WHEEL_BASE = 0.57785;    // meters
        public static final double MAX_VELOCITY = 4.5;      // meters per second
    }   // drive constants


    public static final int timeout = 30;
}
    