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
        encoder_units *= ModuleConstants.POSITION_TO_ANGLE; 
        return encoder_units;
    }   // convert native pivot position to non-absolute angle of wheel

    public static double pivot_toNative(double degrees) {
        SmartDashboard.putNumber("pton", degrees);
        degrees /= ModuleConstants.POSITION_TO_ANGLE;
        degrees *= ModuleConstants.PIVOT_GEAR_RATIO;
        SmartDashboard.putNumber("pton2", degrees);
        return degrees;
    }   // convert angle of wheel to non-absolute angle of wheel

    public static double angle_toAbsolute(double degrees) {
        degrees %= 360;
        degrees += 360;
        degrees %= 360;
        return degrees;
    }   // convert non-absolute angle to absolute angle

    public static double degree_operator(double current, double desired) {
        double error = current-desired;
        double sign = -Math.signum(error);
        double abs = Math.abs(error);
        if (abs>180) {error = (360-abs)*(sign);}
        return error;
    }   // convert current and desired angle to an angle that can be added to current to get to desired

    public static double kinematicsToAngle(double current, double desired) {
        double absolute = angle_toAbsolute(current);
        desired = ( 450 + desired ) %  360;
        current += degree_operator(absolute, desired);
        SmartDashboard.putNumber("desired angle", desired);
        return current;
    }   // convert the kinematics angle to angle of the wheel. This should be reprogrammmed if possible
}