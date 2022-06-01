package frc.robot;

public class Gains {
    public double kp;
    public double ki;
    public double kd;
    public double kf;
    public double kizone;

    public Gains(double kp, double ki, double kd, double kf, double kizone) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.kizone = kizone;
    }
}
