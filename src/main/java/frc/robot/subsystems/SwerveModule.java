// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final CANcoder m_turningEncoderNew;
  
  private final int turningEncoderReversed;
  private final int driveEncoderReversed;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannels,
      Boolean driveEncoderReversedBool,
      Boolean turningEncoderReversedBool) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_turningEncoderNew = new CANcoder(0);
    
    turningEncoderReversed = (turningEncoderReversedBool == true) ? -1 : 1;
    driveEncoderReversed = (driveEncoderReversedBool == true) ? -1 : 1;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        //m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
        (driveEncoderReversed * m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation), new Rotation2d(m_turningEncoderNew.getPosition().getValueAsDouble()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        //m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
        (m_driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation), new Rotation2d(m_turningEncoderNew.getPosition().getValueAsDouble()));
  }


  //used for updating the simulated encoder of the robot
  /*public void updateSimEncoders(SwerveModuleState newState){

    m_turningEncoderSim.setDistance(newState.angle.getRadians());
    m_driveEncoderSim.setRate(newState.speedMetersPerSecond);
    m_driveEncoderSim.setDistance(m_driveEncoderSim.getDistance() + (newState.speedMetersPerSecond * 0.02));
  }*/

  public double getTurnAngle(){
    //return m_turningEncoder.getDistance();
    return m_turningEncoderNew.getPosition().getValueAsDouble();
  }
  public double getSpeed(){
    return m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation;
  }
  public double getModuleDistance(){
    //return m_driveEncoder.getDistance();
    return m_driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoderNew.getPosition().getValue());
    
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
    
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    //update the simulated encoders and gyros
    /*if(Robot.isReal()){
      updateSimEncoders(state);
    }*/
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoderNew.getPosition().getValueAsDouble(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
    
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setPosition(0);
    m_turningEncoderNew.setPosition(0);
  }
}
