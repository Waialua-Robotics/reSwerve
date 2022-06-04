package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class Conversions {

    public static double drive_toVelocity(double raw_encoder_per_100msec) {
        raw_encoder_per_100msec /= ModuleConstants.DRIVE_GEAR_RATIO;
        raw_encoder_per_100msec *= ModuleConstants.ENCODER_TO_VELOCITY;
        return raw_encoder_per_100msec;
    }   // convert native drive velocity to meters-per-second of wheel

    public static double drive_toNative(double meters_per_second) {
        meters_per_second /= ModuleConstants.ENCODER_TO_VELOCITY;
        meters_per_second *= ModuleConstants.DRIVE_GEAR_RATIO;
        return meters_per_second;
    }   // convert meters-per-second of wheel to native drive velocity

    public static double pivot_toDegrees(double encoder_units) {
        encoder_units /= ModuleConstants.PIVOT_GEAR_RATIO;
        encoder_units *= ModuleConstants.TICKS_PER_DEGREE; 
        return encoder_units;
    }   // cancoder abs to motor enconder value


    public static double pivot_toTicks(double degrees) {
        SmartDashboard.putNumber("pton", degrees);
        degrees *= ModuleConstants.TICKS_PER_DEGREE;
        degrees *= ModuleConstants.PIVOT_GEAR_RATIO;
        SmartDashboard.putNumber("pton2", degrees);
        return degrees;
    }  // FX Tick

    public static double angle_toAbsolute(double degrees) {
        degrees %= 360;
        degrees += 360;
        degrees %= 360;
        return degrees;
    }   // convert non-absolute angle to absolute angle

    public static double possitiveNegitive180_to360 (double desired) {
        if (desired < 0) {
            desired += 360;
        }
        return desired;
    }

    public static double FXDesired (double current, double desired, double FXTicks) {
        desired =  Conversions.possitiveNegitive180_to360(desired);
        double error = current-desired; 
        error = pivot_toTicks(error);
        FXTicks += error;
        //desired = ( 450 + desired ) %  360;
        SmartDashboard.putNumber("desired angle", desired);
        return FXTicks;
    }   // convert the kinematics angle to angle of the wheel. This should be reprogrammmed if possible
}