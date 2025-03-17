// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

public class LadderSubsystem extends SubsystemBase {
  public double m_targetPosition = 0;

  public final WPI_TalonSRX m_ladderMotor1 = new WPI_TalonSRX(OIConstants.ladderMotorPort1);
  private final WPI_TalonSRX m_ladderMotor2 = new WPI_TalonSRX(OIConstants.ladderMotorPort2);
  private final double m_encoderTickPerMeter = 1;
  private final double m_meterPerEncoderTick = 1/m_encoderTickPerMeter;

  DoublePublisher ladderHeightPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Lift Height").publish();

//12.75
  public LadderSubsystem(){
    m_ladderMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_ladderMotor1.setSelectedSensorPosition(0);

    m_ladderMotor1.setSensorPhase(true);
    m_ladderMotor2.setSensorPhase(true);

    m_ladderMotor2.setInverted(true);
    m_ladderMotor1.setInverted(true);
    m_ladderMotor1.configPeakOutputForward(1);
    m_ladderMotor2.configPeakOutputForward(1);
    m_ladderMotor2.configPeakCurrentDuration(3000);

    m_ladderMotor1.config_kP(0, 0.05);
    m_ladderMotor1.config_kI(0, 0);
    m_ladderMotor1.config_kD(0, 0.05);
    m_ladderMotor1.config_kF(0, 0);
 
    m_ladderMotor1.configAllowableClosedloopError(0, 100);
    m_ladderMotor2.follow(m_ladderMotor1);
  }

  public void moveToHeight(double position){
    m_targetPosition = position;
  }
  public double getHeight(){
    return m_ladderMotor1.getSelectedSensorPosition() * m_meterPerEncoderTick;
  }
  public double getTargetHeight(){
    return m_targetPosition;
  }
  @Override
  public void periodic() {
    m_ladderMotor1.set(ControlMode.Position, m_targetPosition * m_encoderTickPerMeter);
    ladderHeightPublisher.set(m_ladderMotor1.getSelectedSensorPosition(), 0);
  }
}
