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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.commands.Ladder.LadderMoveToPosition;
import frc.robot.commands.Constructors.PathMaker;
import frc.robot.commands.Constructors.ReefMoveAndLift;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
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
  public double ladderTargetHeight = 0;
  public ShuffleboardTab tab;
  public PathMaker pathMaker = new PathMaker();
  public ReefMoveAndLift reefPathMaker = new ReefMoveAndLift();

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
                    ((m_driverController.getRawAxis(5) * 0.5) - (m_driverController.getRawAxis(4) * 0.5)) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,                    
                    false),
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
    //new POVButton(m_driverController, 90).whileTrue(new IntakeOut(m_intake));//new LadderMoveToPosition(m_ladder, ladderTargetHeight));
    new JoystickButton(m_driverController, 2).whileTrue(new IntakeIn(m_intake)); //B Button on Xbox
    new JoystickButton(m_driverController, 6).onTrue(new IntakeIn(m_intake)); //B Button on Xbox
    new JoystickButton(m_driverController, 3).whileTrue(reefPathMaker.createCommand(1, 1, m_ladder, m_robotDrive, m_intake)); //Top Right Button on Xbox
  
    //Left L1
    //new JoystickButton(m_driverRJoystick, 0).whileTrue(reefPathMaker.createCommand(1, 1, m_ladder, m_robotDrive, m_intake);
    //Left L2
    //new JoystickButton(m_driverRJoystick, 0).whileTrue(reefPathMaker.createCommand(1, 2, m_ladder, m_robotDrive, m_intake);
    //Left L3
    //new JoystickButton(m_driverRJoystick, 0).whileTrue(reefPathMaker.createCommand(1, 3, m_ladder, m_robotDrive, m_intake);
    //Right L1
    //new JoystickButton(m_driverRJoystick, 0).whileTrue(reefPathMaker.createCommand(2, 1, m_ladder, m_robotDrive, m_intake);
    //Right L2
    //new JoystickButton(m_driverRJoystick, 0).whileTrue(reefPathMaker.createCommand(2, 2, m_ladder, m_robotDrive, m_intake);
    //Right L3
    //new JoystickButton(m_driverRJoystick, 0).whileTrue(reefPathMaker.createCommand(2, 3, m_ladder, m_robotDrive, m_intake);
    
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
    SmartDashboard.putData("Lift Zero", new LadderMoveToPosition(m_ladder, 1000));
    SmartDashboard.putData("Intake In", new IntakeIn(m_intake));
    SmartDashboard.putData("Intake Out", new IntakeOut(m_intake));
    //SmartDashboard.putData(reefPathMaker.createCommand(1, 1, m_ladder, m_robotDrive, m_intake, m_robotDrive.m_vision.getReefTransform()));
  }
  public void updateDashboard(){
    SmartDashboard.putNumber("Lift Height", m_ladder.getHeight());
    SmartDashboard.putNumber("Lift Target Height", m_ladder.m_targetPosition);
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
  }
  public void periodic() {
    updateDashboard();

    if(m_robotDrive.m_vision.cameraConnected){
      m_robotDrive.m_vision.getReefTransform();
    }

    if(Timer.getMatchTime() < 3 && Timer.getMatchTime() != -1){
      CommandScheduler.getInstance().schedule(new LadderMoveToPosition(m_ladder, Constants.LadderConstants.zeroPosition));
      SmartDashboard.putString("Status", "Auto Retracting Ladder");
    }
    //m_robotDrive.changeMaxSpeed(m_driverRJoystick.getRawAxis(0));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int autoNumber) {
  //   if (Timer.getMatchTime() > 13) {
  //   m_robotDrive.m_frontLeft.m_driveMotor.set(0.4);
  //   m_robotDrive.m_frontLeft.m_turningMotor.set(autoNumber);
  // }
    ParallelRaceGroup cmd = new RunCommand(()->m_robotDrive.drive(2,0,0,false)).withTimeout(3);
   return cmd;
  }
    // return PathMaker.createPathGlobal(
    //     m_robotDrive,
    //     new Pose2d(1, 0, new Rotation2d(0)),
    //     List.of());
    // }  

      
      /*.andThen(pathMaker.createPath(
      m_robotDrive,
      new Pose2d(-3,-3, new Rotation2d(0)),
      List.of(new Translation2d(-3,0)),
      false));
      */
  
  public Command getTestCommand(int testNumber){
    switch(testNumber){
      case 1: 
        return pathMaker.createPath(
          m_robotDrive,
          new Pose2d(5, 0, new Rotation2d(0)),
          List.of(),
          true
        )
        .andThen((new IntakeOut(m_intake)).withTimeout(3))
        //.withTimeout(5)
        .andThen(pathMaker.createPath(
          m_robotDrive,
          new Pose2d(5, 5, new Rotation2d(0)),
          List.of(),
          true
        ));
      case 2:
        return reefPathMaker.createCommand(1, 1, m_ladder, m_robotDrive, m_intake);
      default:
        return new Command() {
          
        };
        
    }
  }
}
