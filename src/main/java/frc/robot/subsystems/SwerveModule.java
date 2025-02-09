// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final CANcoder m_turningEncoderNew;
  
  private final int turningEncoderReversed;
  private final int driveEncoderReversed;



  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0.5, 0.5);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0.2,
          0.2,
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
    m_turningEncoderNew = new CANcoder(turningEncoderChannels);
    
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
   * 
   */
  public void setMotionMagicConfigs(){
    var talonFXConfigs = new TalonFXConfiguration();
    var slotConfigs = talonFXConfigs.Slot0;
    slotConfigs.kS = 0.25;
    slotConfigs.kV = 0.12;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    //motionMagicConfigs.
  }


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
    System.out.println(m_driveMotor.getPosition().getValue());
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
    desiredState.speedMetersPerSecond *= driveEncoderReversed;
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);
    
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    

    //update the simulated encoders and gyros
    /*if(Robot.isReal()){
      updateSimEncoders(state);
    }*/
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation, desiredState.speedMetersPerSecond);

    double pos = m_turningEncoderNew.getPosition().getValueAsDouble();
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = //desiredState.angle.getRotations() - encoderRotation.getRotations();
      m_turningPIDController.calculate(m_turningEncoderNew.getAbsolutePosition().getValueAsDouble() * 2* Math.PI, desiredState.angle.getRadians());
    
    //final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    //m_driveMotor.setControl(m_request.withVelocity(desiredState.speedMetersPerSecond/ModuleConstants.kDriveEncoderDistancePerRotation));
    // Calculate the turning motor output from the turning PID controller.
    //System.out.println(desiredState.speedMetersPerSecond);
    m_driveMotor.setVoltage(desiredState.speedMetersPerSecond / DriveConstants.kvVoltSecondsPerMeter);//driveOutput);
    m_turningMotor.setVoltage(turningEncoderReversed * turnOutput);
    
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setPosition(0);
    m_turningEncoderNew.setPosition(0);
  }
}
