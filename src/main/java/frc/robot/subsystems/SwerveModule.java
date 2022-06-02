// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Conversions;

public class SwerveModule extends SubsystemBase {

  private TalonFX m_drive;
  private TalonFX m_pivot;
  private CANCoder m_encoder;

  public double initial_angle;

  private static Gains drivePID = new Gains(0.01,0,0,0,0);
  private static Gains pivotPID = new Gains(0.05,0,0,0,0);

  public SwerveModule( int driveID,
                       int pivotID,
                       int encoderID,
                       boolean reverseDrive ) {

      m_pivot = new TalonFX(pivotID);
      m_drive = new TalonFX(driveID);
      m_encoder = new CANCoder(encoderID);

      m_pivot.configFactoryDefault(Constants.timeout);
      m_drive.configFactoryDefault(Constants.timeout);

      m_pivot.setInverted(true);
      m_drive.setInverted(reverseDrive);

      m_pivot.setNeutralMode(NeutralMode.Brake);
      m_drive.setNeutralMode(NeutralMode.Brake);

      m_pivot.config_kP(0, pivotPID.kp, Constants.timeout);
      m_pivot.config_kI(0, pivotPID.ki, Constants.timeout);
      m_pivot.config_kD(0, pivotPID.kd, Constants.timeout);
      m_pivot.config_kF(0, pivotPID.kf, Constants.timeout);
      
      m_drive.config_kP(0, drivePID.kp, Constants.timeout);
      m_drive.config_kI(0, drivePID.ki, Constants.timeout);
      m_drive.config_kD(0, drivePID.kd, Constants.timeout);
      m_drive.config_kF(0, drivePID.kf, Constants.timeout);

      m_drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeout);
      m_drive.setSelectedSensorPosition(0);
      m_pivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeout);
      m_pivot.setSelectedSensorPosition(Conversions.pivot_toNative(getEncoder()));
      SmartDashboard.putNumber("initial angle", Conversions.pivot_toNative(getEncoder()));
  }

          private double getEncoder() {
            return m_encoder.getAbsolutePosition();
          }

          private void setAngle(double angle) {
            angle = Conversions.pivot_toNative(angle);
            m_pivot.set(ControlMode.Position, angle);
          }

          private double getAngle() {
            // double angle = m_pivot.getSelectedSensorPosition();
            // double temp = m_encoder.getAbsolutePosition() - Conversions.pivot_toDegrees(angle);
            // SmartDashboard.putNumber("debug differnece", temp);
            // return Conversions.pivot_toDegrees(angle);
            return getEncoder();
          }

          private void setVelocity(double velocity) {
            velocity = Conversions.drive_toNative(velocity);
            m_drive.set(ControlMode.Velocity, velocity);
          }

          private double getVelocity() {
            double velocity = m_drive.getSelectedSensorVelocity();
            return Conversions.drive_toVelocity(velocity);
          }

  public SwerveModuleState getState() {
    Rotation2d angle = Rotation2d.fromDegrees( Conversions.angle_toAbsolute( getAngle() ) );
    double speedMetersPerSecond = getVelocity();
    return new SwerveModuleState(speedMetersPerSecond, angle);
  }

  private int i = 0;

  public void setState(SwerveModuleState state) {
    i++;
    SmartDashboard.putNumber("iterator", i);
    SmartDashboard.putNumber("debugging for speed", state.speedMetersPerSecond);
    if ( Math.abs( state.speedMetersPerSecond ) > 0.1 ) {
        setAngle( Conversions.kinematicsToAngle( getAngle(), state.angle.getDegrees() ) );
        setVelocity( state.speedMetersPerSecond );
    } else {
        stop(); 
    }
  }

  public void stop() {
    setVelocity(0);
    setAngle(0);
  }

  @Override
  public void periodic() {

  }
}
