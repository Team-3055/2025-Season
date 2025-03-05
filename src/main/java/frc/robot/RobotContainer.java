// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.commands.autoCommands.PathMaker;
import frc.robot.commands.autoCommands.Tests.MoveForward;
import frc.robot.commands.autoCommands.Tests.ZeroModules;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PathMaker pathMaker = new PathMaker();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                    // converting them to actual units.
                    Math.abs(m_driverController.getLeftX()) + Math.abs(m_driverController.getLeftY()) > 0.25 ? - m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond : 0,
                    Math.abs(m_driverController.getLeftX()) + Math.abs(m_driverController.getLeftY()) > 0.25 ? - m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond : 0,
                    ((m_driverController.getRawAxis(5) * 0.5) - (m_driverController.getRawAxis(4) * 0.5)) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,                    
                    //m_driverController.getRawAxis(2) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    false),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new Command() {
      
    };
    /*pathMaker.createPath(
      m_robotDrive,
      new Pose2d(0.5, 0, new Rotation2d(0)),
      List.of(),//new Translation2d(3,0)),
      false
    );/*.andThen(pathMaker.createPath(
      m_robotDrive,
      new Pose2d(-3,-3, new Rotation2d(0)),
      List.of(new Translation2d(-3,0)),
      false));
      */
  }

  public Command getTestCommand(int testNumber){
    switch(testNumber){
      case 1: 
        return pathMaker.createPath(
          m_robotDrive,
          new Pose2d(4, 0, new Rotation2d(0)),
          List.of(),//new Translation2d(3,0)),
          false);
      default:
        return new Command() {
          
        };
        
    }
  }
}

