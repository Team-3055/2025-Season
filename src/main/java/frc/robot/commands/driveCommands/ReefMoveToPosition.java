// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Ladder.LadderMoveToPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LadderSubsystem;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class ReefMoveToPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LadderSubsystem m_ladder;
  private final DriveSubsystem m_drive;
  private final Intake m_intake;
  private int m_stalk = 0;
  private int m_level = 0;
  private Command finalCommands;
  private final Transform2d tagTorobotLeftStalkPosiion = new Transform2d(1,-0.127, new Rotation2d(0));
  private final Transform2d tagTorobotRightStalkPosiion = new Transform2d(-0.127,-0.127, new Rotation2d(0));

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public ReefMoveToPosition(int stalk, int level, LadderSubsystem ladder, DriveSubsystem drive, Intake intake) {
    m_ladder = ladder;
    m_drive = drive;
    m_intake = intake;
    m_stalk = stalk;
    m_level = level;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ladder);
    addRequirements(drive);
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().cancel(finalCommands);
    if(m_drive.m_vision.cameraConnected){
            var robotToTag = m_drive.m_vision.getReefTransform();
            if(robotToTag != null){
                //Left Stalk
                //L2
                if(m_stalk == 1 && m_level == 1){
                    var robotToPosition = robotToTag.plus(tagTorobotLeftStalkPosiion);
                    finalCommands = new MoveToPosition(m_drive, new Pose2d().plus(robotToPosition), List.of(), false)
                    .withTimeout(5)
                    .alongWith(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.bottomStalkPosition))
                    .andThen(new IntakeOut(m_intake).withTimeout(3));
                    System.out.println(robotToPosition);
                }
                //L3
                else if(m_stalk == 1 && m_level == 2){
                    var robotToPosition = robotToTag.plus(tagTorobotLeftStalkPosiion);
                    finalCommands = new MoveToPosition(m_drive, new Pose2d().plus(robotToPosition), List.of(), false)
                    .withTimeout(5)
                    .alongWith(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.middleStalkPosition))
                    .andThen(new IntakeOut(m_intake).withTimeout(3));
                    System.out.println(robotToPosition);

                }
                //L4
                else if(m_stalk == 1 && m_level == 3){
                    var robotToPosition = robotToTag.plus(tagTorobotLeftStalkPosiion);
                    finalCommands = new MoveToPosition(m_drive, new Pose2d().plus(robotToPosition), List.of(), false)
                    .alongWith(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.topStalkPosition))
                    .andThen(new IntakeOut(m_intake).withTimeout(3));
                }

                //Right Stalk
                //L2
                else if(m_stalk == 2 && m_level == 1){
                    var robotToPosition = robotToTag.plus(tagTorobotRightStalkPosiion);
                    finalCommands = new MoveToPosition(m_drive, new Pose2d().plus(robotToPosition), List.of(), false)
                    .alongWith(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.bottomStalkPosition))
                    .andThen(new IntakeOut(m_intake).withTimeout(3));
                    System.out.println(robotToPosition);

                } 
                //L3
                else if(m_stalk == 2 && m_level == 2){
                    var robotToPosition = robotToTag.plus(tagTorobotRightStalkPosiion);
                    finalCommands = new MoveToPosition(m_drive, new Pose2d().plus(robotToPosition), List.of(), false)
                    .alongWith(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.middleStalkPosition))
                    .andThen(new IntakeOut(m_intake).withTimeout(3));
                }
                //L4
                else if(m_stalk == 2 && m_level == 3){
                    var robotToPosition = robotToTag.plus(tagTorobotRightStalkPosiion);
                    finalCommands = new MoveToPosition(m_drive, new Pose2d().plus(robotToPosition), List.of(), false)
                    .alongWith(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.topStalkPosition))
                    .andThen(new IntakeOut(m_intake).withTimeout(3));
                }
              CommandScheduler.getInstance().schedule(finalCommands);
            }
            SmartDashboard.putString("Status", "No Tag Found");
        }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(finalCommands);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
