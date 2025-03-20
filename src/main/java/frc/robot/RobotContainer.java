// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.commands.Ladder.LadderMoveToPosition;
import frc.robot.commands.Constructors.PathMaker;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
  private final LadderSubsystem m_ladder = new LadderSubsystem();
  private final Intake m_intake = new Intake(); 
  public double ladderTargetHeight = 0;
  public ShuffleboardTab tab;
  public PathMaker pathMaker = new PathMaker();
  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_driverRJoystick = new Joystick(OIConstants.kRightJoystickPort);

  DoublePublisher ladderHeightPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Lift Height").publish();
  DoublePublisher targetHeightPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Target lift Height").publish();

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
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //
    //new JoystickButton(m_driverRJoystick,5).whileTrue(new LadderMoveToPosition(m_ladder, 9000)); //Top Left Button > Top Stick
    //new JoystickButton(m_driverRJoystick,3).whileTrue(new LadderMoveToPosition(m_ladder, 6000)); //Bottom Left > Middle Stick
    //new JoystickButton(m_driverRJoystick,4).whileTrue(new LadderMoveToPosition(m_ladder, 3000)); //Bottom Right > Bottom Stick
    new JoystickButton(m_driverRJoystick,6).whileTrue(new LadderMoveToPosition(m_ladder, 3000)); //Top Right > Intake*/
    new JoystickButton(m_driverRJoystick,2).whileTrue(new LadderMoveToPosition(m_ladder, 1000)); //Thumb Button 

    new JoystickButton(m_driverController, 3).whileTrue(new IntakeIn(m_intake)); //X Button on Xbox
    new JoystickButton(m_driverController, 2).whileTrue(new IntakeOut(m_intake)); //B Button on Xbox



    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_driverController.getRawButton(0)).onTrue();

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b(0).whileTrue();
  }
  public void updateNetworkTables(){
    ladderHeightPublisher.set(m_ladder.getHeight());
    targetHeightPublisher.set(m_ladder.m_targetPosition);
  }
  public void periodic() {
    updateNetworkTables();
    //m_robotDrive.changeMaxSpeed(m_driverRJoystick.getRawAxis(0));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return PathMaker.createPathLocal(
      m_robotDrive,
      new Transform2d(-1, 0, new Rotation2d(0)),
      List.of());/*.andThen(pathMaker.createPath(
      m_robotDrive,
      new Pose2d(-3,-3, new Rotation2d(0)),
      List.of(new Translation2d(-3,0)),
      false));
      */
  }

  public Command getTestCommand(int testNumber){
    switch(testNumber){
      case 1: 
        return pathMaker.createPathLocal(
          m_robotDrive,
          new Transform2d(2, 2, new Rotation2d(0)),
          List.of());/* .andThen(pathMaker.createPath(
          m_robotDrive, 
          new Pose2d(0,2, new Rotation2d(0)),
          List.of(),
          true)
        );*/
      case 2:
        return new LadderMoveToPosition(m_ladder, 5000).withTimeout(5).andThen(new LadderMoveToPosition(m_ladder, 0));
      default:
        return new Command() {
          
        };
        
    }
  }
}

