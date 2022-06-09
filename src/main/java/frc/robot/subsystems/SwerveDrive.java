// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDrive extends SubsystemBase {

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveModule FL;
  private SwerveModule FR;
  private SwerveModule RL;
  private SwerveModule RR;

  public SwerveModuleState[] states;

  private static final double HalfWidth = DriveConstants.TRACK_WIDTH / 2; // chassis width
  private static final double HalfLength = DriveConstants.WHEEL_BASE / 2;   // chassis length
     
  public SwerveDrive() {
     FL = new SwerveModule(ID.FLdrive, ID.FLpivot, ID.FLencoder, true);
     FR = new SwerveModule(ID.FRdrive, ID.FRpivot, ID.FRencoder, true);
     RL = new SwerveModule(ID.RLdrive, ID.RLpivot, ID.RLencoder, false);
     RR = new SwerveModule(ID.RRdrive, ID.RRpivot, ID.RRencoder, false);
       }

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //was -+, ++, --, +-
        new Translation2d( HalfWidth,  HalfLength),
        new Translation2d( HalfWidth,  -HalfLength),
        new Translation2d(-HalfWidth, HalfLength),
        new Translation2d( -HalfWidth, -HalfLength)
  );

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics, m_gyro.getRotation2d());

  /*public Rotation2d getYaw() {
    return m_gyro.getAngle;
  }*/

  public void zeroYaw() {
      m_gyro.reset();
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putBoolean("field Relative", fieldRelative);
      rot = 0;  // negate all rotation
      states = kinematics.toSwerveModuleStates(
            fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, rot, m_gyro.getRotation2d()) 
              : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );  
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_VELOCITY);
      // order of motores FR, FL, RL, RR
      FL.setDesiredState(states[0]); 
      FR.setDesiredState(states[1]); 
      RL.setDesiredState(states[2]); 
      RR.setDesiredState(states[3]); 
      SmartDashboard.putString("setFL", states[0].toString()); 
      SmartDashboard.putString("setFR", states[1].toString()); 
      SmartDashboard.putString("setRL", states[2].toString());
      SmartDashboard.putString("setRR", states[3].toString()); 
  } // query the kinematics for desired module states and set the modules to their corresponding state

  public void stop() {
    FL.stop();
    FR.stop();
    RL.stop();
    RR.stop();
  } // calls stop on modules

  public double heading() {
    return m_gyro.getAngle() % 360;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, m_gyro.getRotation2d());
  }



  @Override
  public void periodic() {
      odometry.update(
          m_gyro.getRotation2d(), 
          FL.getState(),
          FR.getState(),
          RL.getState(),
          RR.getState()
      );  // refresh module states

      SmartDashboard.putNumber("FR Velocity", FR.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("FR Angle", FR.getState().angle.getDegrees() );
      SmartDashboard.putNumber("FL Velocity", FL.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("FL Angle", FL.getState().angle.getDegrees() );
      SmartDashboard.putNumber("RL Velocity", RL.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("RL Angle", RL.getState().angle.getDegrees() );
      SmartDashboard.putNumber("RR Velocity", RR.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("RR Angle", RR.getState().angle.getDegrees() );

      SmartDashboard.putString("odometry", odometry.getPoseMeters().toString());
      SmartDashboard.putNumber("Yaw", heading());

      SmartDashboard.putNumber("FL Initial Angle", FL.encoderAngle);
      SmartDashboard.putNumber("FR Initial Angle", FR.encoderAngle);
      SmartDashboard.putNumber("RL Initial Angle", RL.encoderAngle);
      SmartDashboard.putNumber("RR Initial Angle", RR.encoderAngle);

  }
}
