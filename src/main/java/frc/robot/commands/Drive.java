// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> get_x;
  private final Supplier<Double> get_y;
  private final Supplier<Double> get_omega;
  private final Supplier<Boolean> field_centric;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(
      SwerveDrive swerveDrive,
      Supplier<Double> get_x, 
      Supplier<Double> get_y, 
      Supplier<Double> get_omega, 
      Supplier<Boolean> field_centric
      ) {

    this.get_x = get_x;
    this.get_y = get_y;
    this.get_omega = get_omega;
    this.field_centric = field_centric;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(
        get_x.get(),
        get_y.get(),
        get_omega.get(),
        field_centric.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}