// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.Tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class MoveForward extends Command {
  /** Creates a new SwerveForwardTest. */
  private final DriveSubsystem m_drive;
  final SwerveModuleState[] desiredStates = {
    new SwerveModuleState(0.1, new Rotation2d(0)),
    new SwerveModuleState(0.1, new Rotation2d(0)),
    new SwerveModuleState(0.1, new Rotation2d(0)),
    new SwerveModuleState(0, new Rotation2d(0))
  };

  public MoveForward(DriveSubsystem driveSubsystem) {
    m_drive = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setModuleStates(desiredStates);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drive.setModuleStates(desiredStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
