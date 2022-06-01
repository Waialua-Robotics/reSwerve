package frc.robot;

import frc.robot.Constants.ModuleConstants;

public class Conversions {

    public static double drive_toVelocity(double raw_encoder_per_100msec) {
        raw_encoder_per_100msec /= ModuleConstants.DRIVE_GEAR_RATIO;
        raw_encoder_per_100msec *= ModuleConstants.ENCODER_TO_VELOCITY;
        return raw_encoder_per_100msec;
    }

    public static double drive_toNative(double meters_per_second) {
        meters_per_second /= ModuleConstants.ENCODER_TO_VELOCITY;
        meters_per_second *= ModuleConstants.DRIVE_GEAR_RATIO;
        return meters_per_second;
    }

    public static double pivot_toDegrees(double encoder_units) {
        encoder_units /= ModuleConstants.PIVOT_GEAR_RATIO;
        encoder_units *= ModuleConstants.POSITION_TO_ANGLE;
        return encoder_units;
    }

    public static double pivot_toNative(double degrees) {
        degrees /= ModuleConstants.POSITION_TO_ANGLE;
        degrees *= ModuleConstants.PIVOT_GEAR_RATIO;
        return degrees;
    }

    public static double angle_toAbsolute(double degrees) {
        degrees %= 360;
        degrees += 360;
        degrees %= 360;
        return degrees;
    }

    public static double degree_operator(double current, double desired) {
        double error = current-desired;
        double sign = -Math.signum(error);
        double abs = Math.abs(error);
        if (abs>180) {error = (360-abs)*(sign);}
        return error;
    }

    public static double kinematicsToAngle(double current, double desired) {
        double absolute = angle_toAbsolute(current);
        desired = ( 450 + desired ) %  360;
        current += degree_operator(absolute, desired);
        return current;
    }
}