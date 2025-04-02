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
import frc.robot.subsystems.Dealgaefier;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.commands.Ladder.LadderMoveToPosition;
import frc.robot.commands.driveCommands.MoveToPosition;
import frc.robot.commands.driveCommands.ReefMoveToPosition;
import frc.robot.commands.Dealgaefier.AlgaeIn;
import frc.robot.commands.Dealgaefier.AlgaeOut;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final Dealgaefier m_dealgae = new Dealgaefier();
  private final PowerDistribution m_PDP = new PowerDistribution();

  public double ladderTargetHeight = 0;
  public ShuffleboardTab tab;

public boolean driverDriveControlEnabled = true;

  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_driverRJoystick = new Joystick(OIConstants.kRightJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
    initDashboard();
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
                    Math.abs(m_driverController.getRawAxis(1)) > 0.05 ? -m_driverController.getRawAxis(1) * DriveConstants.kMaxSpeedMetersPerSecond : 0,
                    Math.abs(m_driverController.getRawAxis(0)) > 0.05 ? -m_driverController.getRawAxis(0) * DriveConstants.kMaxSpeedMetersPerSecond : 0,
                    Math.abs(m_driverController.getRawAxis(4)) > 0.2 ? -m_driverController.getRawAxis(4) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond: 0,                    
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
    new POVButton(m_driverController,90).whileTrue(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.bottomStalkPosition)); //Bottom Right > Bottom Stick
    new POVButton(m_driverController,180).whileTrue(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.zeroPosition)); //Top Right > Intake*/
    new POVButton(m_driverController,0).whileTrue(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.middleStalkPosition)); //Top Right > Intake*/
   
    new POVButton(m_driverController, 90).whileTrue(new IntakeOut(m_intake));//new LadderMoveToPosition(m_ladder, ladderTargetHeight));

    //new JoystickButton(m_driverController, 1).whileTrue(null);//A Button on Xbox    
    new JoystickButton(m_driverController, 1).whileTrue(new IntakeIn(m_intake)); //X Button on Xbox    
    new JoystickButton(m_driverController, 2).whileTrue(new AlgaeIn(m_dealgae)); //B Button on Xbox    
    new JoystickButton(m_driverController, 3).whileTrue(new AlgaeOut(m_dealgae)); //Yt Button on Xbox    
    new JoystickButton(m_driverController, 4).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro(), m_robotDrive));

    //Left L1
    new JoystickButton(m_driverRJoystick, 0).whileTrue(new ReefMoveToPosition(1, 1, m_ladder, m_robotDrive, m_intake));
    //Left L2
    new JoystickButton(m_driverRJoystick, 0).whileTrue(new ReefMoveToPosition(1, 2, m_ladder, m_robotDrive, m_intake));
    //Left L3
    new JoystickButton(m_driverRJoystick, 0).whileTrue(new ReefMoveToPosition(1, 3, m_ladder, m_robotDrive, m_intake));
    //Right L1
    new JoystickButton(m_driverRJoystick, 0).whileTrue(new ReefMoveToPosition(2, 1, m_ladder, m_robotDrive, m_intake));
    //Right L2
    new JoystickButton(m_driverRJoystick, 0).whileTrue(new ReefMoveToPosition(2, 2, m_ladder, m_robotDrive, m_intake));
    //Right L3
    new JoystickButton(m_driverRJoystick, 0).whileTrue(new ReefMoveToPosition(2, 3, m_ladder, m_robotDrive, m_intake));
    
    /*new JoystickButton(m_driverController, 1).onTrue(
     PathMaker.createPathLocal(m_robotDrive, new Transform2d(),List.of()) 
    );*/


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_driverController.getRawButton(0)).onTrue();

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b(0).whileTrue();
  }

  public void initDashboard(){
    SmartDashboard.putData("Lift L1",  new LadderMoveToPosition(m_ladder, Constants.LadderConstants.zeroPosition));
    SmartDashboard.putData("Lift L2",  new LadderMoveToPosition(m_ladder, Constants.LadderConstants.bottomStalkPosition));
    SmartDashboard.putData("Lift L3",  new LadderMoveToPosition(m_ladder, Constants.LadderConstants.middleStalkPosition));
    SmartDashboard.putData("Lift L4",  new LadderMoveToPosition(m_ladder, Constants.LadderConstants.topStalkPosition));

    SmartDashboard.putData("Intake In", new IntakeIn(m_intake));
    SmartDashboard.putData("Intake Out", new IntakeOut(m_intake));

    SmartDashboard.putData("Reef Left L2", new ReefMoveToPosition(1, 1, m_ladder, m_robotDrive, m_intake));
    SmartDashboard.putData("Reef Left L3", new ReefMoveToPosition(1, 2, m_ladder, m_robotDrive, m_intake));
    SmartDashboard.putData("Reef Left L4", new ReefMoveToPosition(1, 3, m_ladder, m_robotDrive, m_intake));
    SmartDashboard.putData("Reef Right L2", new ReefMoveToPosition(2, 1, m_ladder, m_robotDrive, m_intake));
    SmartDashboard.putData("Reef Right L3", new ReefMoveToPosition(2, 2, m_ladder, m_robotDrive, m_intake));
    SmartDashboard.putData("Reef Right L4", new ReefMoveToPosition(2, 3, m_ladder, m_robotDrive, m_intake));
  }
  public void updateDashboard(){
    SmartDashboard.putData("PDP Data", m_PDP);
    SmartDashboard.putNumber("Lift Height", m_ladder.getHeight());
    SmartDashboard.putNumber("Lift Target Height", m_ladder.m_targetPosition);
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putData("Drive Subsystem", m_robotDrive);
    SmartDashboard.putData("Ladder Subsystem", m_ladder);
    SmartDashboard.putData("Intake Subsystem", m_intake);
    SmartDashboard.putData("Dealgaefier Subsystem", m_dealgae);
  }
  public void periodic() {
    updateDashboard();
    //m_robotDrive.changeMaxSpeed(m_driverRJoystick.getRawAxis(0));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int autoNumber) {
    return new LadderMoveToPosition(m_ladder, 10);
    //return (new MoveToPosition(m_robotDrive, new Pose2d(0,1,new Rotation2d()), List.of(), false).andThen(new IntakeIn(m_intake).withTimeout(2)))
    //  .andThen(new MoveToPosition(m_robotDrive, new Pose2d(1,0,new Rotation2d()), List.of(), false));


  //   if (Timer.getMatchTime() > 13) {
  //   m_robotDrive.m_frontLeft.m_driveMotor.set(0.4);
  //   m_robotDrive.m_frontLeft.m_turningMotor.set(autoNumber);
  // }   
  /*return pathMaker.createPath(
    m_robotDrive,
    new Pose2d(1, 0, new Rotation2d(0)),
    List.of(),
    false);*/
    //.andThen(pathMaker.createPath(
    //m_robotDrive,
    //new Pose2d(0,0.5, new Rotation2d(0)),
    //List.of(),
    //false));
  }  
  //  ParallelRaceGroup cmd = new RunCommand(()->m_robotDrive.drive(2,0,0,false)).withTimeout(3);
  // return cmd;
    
      
      /*.andThen(pathMaker.createPath(
      m_robotDrive,
      new Pose2d(-3,-3, new Rotation2d(0)),
      List.of(new Translation2d(-3,0)),
      false));
      */
  
  public Command getTestCommand(int testNumber){
    switch(testNumber){
      case 1: 
        return new MoveToPosition(
          m_robotDrive,
          new Pose2d(0.5, -0.5, new Rotation2d(0)),
          List.of(),
          false
        );
      case 2:
        return new ReefMoveToPosition(1, 1, m_ladder, m_robotDrive, m_intake);//reefPathMaker.createCommand(1, 1, m_ladder, m_robotDrive, m_intake);
      case 3:
        return new LadderMoveToPosition(m_ladder, 10);
      default:
        return new Command() {
          
        };
        
    }
  }
}
