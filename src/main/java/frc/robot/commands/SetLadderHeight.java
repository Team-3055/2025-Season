package frc.robot.commands;

import frc.robot.subsystems.LadderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetLadderHeight extends Command {
    private final LadderSubsystem m_subsystem;

    public SetLadderHeight(LadderSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void setLadderHeight(int height) {
    m_subsystem.setHeight(height);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
