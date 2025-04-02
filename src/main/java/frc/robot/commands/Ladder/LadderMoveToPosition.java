// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Ladder;

import frc.robot.subsystems.LadderSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LadderMoveToPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final LadderSubsystem m_ladder;
  private double y_val;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LadderMoveToPosition(LadderSubsystem subsystem, double yVal) {
    m_ladder = subsystem;
    y_val = yVal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ladder.moveToHeight(y_val);
    System.out.println("Set");

    //System.out.println(m_ladder.m_ladderMotor1.getSelectedSensorPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_ladder.moveToHeight(0);
    //m_ladder.moveToHeight(Constants.OIConstants.zeroPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
