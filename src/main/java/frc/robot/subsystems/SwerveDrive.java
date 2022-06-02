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

public class SwerveDrive extends SubsystemBase {

  private SwerveModule FL;
  private SwerveModule FR;
  private SwerveModule RL;
  private SwerveModule RR;

  private SwerveModuleState[] states;

  private static final double HalfWidth = DriveConstants.TRACK_WIDTH / 2; // chassis width
  private static final double HalfLength = DriveConstants.WHEEL_BASE / 2;   // chassis length
     
  public SwerveDrive() {
      FL = new SwerveModule(ID.FLdrive, ID.FLpivot, ID.FLencoder, false);
      FR = new SwerveModule(ID.FRdrive, ID.FRpivot, ID.FRencoder, true);
      RL = new SwerveModule(ID.RLdrive, ID.RLpivot, ID.RLencoder, false);
      RR = new SwerveModule(ID.RRdrive, ID.RRpivot, ID.RRencoder, true);
  }

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(-HalfWidth,  HalfLength),
        new Translation2d( HalfWidth,  HalfLength),
        new Translation2d(-HalfWidth, -HalfLength),
        new Translation2d( HalfWidth, -HalfLength)
  );

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics, 
        Rotation2d.fromDegrees( getYaw().getDegrees() )
  );

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees( 0 );
  }

  public void zeroYaw() {}

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      rot = 0;
      states = kinematics.toSwerveModuleStates(
            fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, rot, getYaw() ) 
              : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );

      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_VELOCITY);
        FL.setState(states[0]);
        FR.setState(states[1]);
        RL.setState(states[2]);
        RR.setState(states[3]);
        SmartDashboard.putString("setFL", states[0].toString());
        SmartDashboard.putString("setFR", states[1].toString());
        SmartDashboard.putString("setRL", states[2].toString());
        SmartDashboard.putString("setRR", states[3].toString());
  }

  public void stop() {
    FL.stop();
    FR.stop();
    RL.stop();
    RR.stop();
  }

  @Override
  public void periodic() {
      odometry.update(
          getYaw(), 
          FL.getState(),
          RL.getState(),
          FR.getState(),
          RR.getState()
      );

      SmartDashboard.putNumber("FR Velocity", FR.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("FR Angle", FR.getState().angle.getDegrees() );
      SmartDashboard.putNumber("FL Velocity", FL.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("FL Angle", FL.getState().angle.getDegrees() );
      SmartDashboard.putNumber("RL Velocity", RL.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("RL Angle", RL.getState().angle.getDegrees() );
      SmartDashboard.putNumber("RR Velocity", RR.getState().speedMetersPerSecond );
      SmartDashboard.putNumber("RR Angle", RR.getState().angle.getDegrees() );

      SmartDashboard.putString("odometry", odometry.getPoseMeters().toString());
      SmartDashboard.putNumber("Yaw", getYaw().getDegrees() );

      SmartDashboard.putNumber("FL Initial Angle", FL.initial_angle);
      SmartDashboard.putNumber("FR Initial Angle", FR.initial_angle);
      SmartDashboard.putNumber("RL Initial Angle", RL.initial_angle);
      SmartDashboard.putNumber("RR Initial Angle", RR.initial_angle);

  }
}
