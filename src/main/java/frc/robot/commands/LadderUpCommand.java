package frc.robot.commands;

import frc.robot.subsystems.LadderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LadderUpCommand extends Command {
  private final LadderSubsystem m_subsystem;
  private double ladder_voltage = 5.0;

  public LadderUpCommand(LadderSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.moveUp(ladder_voltage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopLadder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
