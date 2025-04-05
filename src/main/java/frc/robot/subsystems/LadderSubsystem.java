// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.LadderConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged
public class LadderSubsystem extends SubsystemBase {
  public double m_targetPosition = 0;
  public boolean disabled = false;

  public double kP = 0.75;
  public double kI = 0.0005;
  public double kD = 0.0005;
  public double kF = 0.02;

  private final SparkMax m_ladderMotor_1 = new SparkMax(LadderConstants.ladderMotorPort1, MotorType.kBrushless);
  private final SparkMax m_ladderMotor_2 = new SparkMax(LadderConstants.ladderMotorPort2, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController;
  private SparkMaxConfig motorConfigMotor1 = new SparkMaxConfig();
  private SparkMaxConfig motorConfigMotor2 = new SparkMaxConfig();

//12.75
  public LadderSubsystem(){
    closedLoopController = m_ladderMotor_1.getClosedLoopController();
    motorConfigMotor1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, kF);
    motorConfigMotor1.idleMode(IdleMode.kBrake);
    motorConfigMotor1.inverted(true);
    motorConfigMotor1.smartCurrentLimit(60);
    
    motorConfigMotor2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, kF);
    motorConfigMotor2.idleMode(IdleMode.kBrake);
    motorConfigMotor2.follow(Constants.LadderConstants.ladderMotorPort1);
    motorConfigMotor2.inverted(true);
    motorConfigMotor2.smartCurrentLimit(60);

    m_ladderMotor_1.configure(motorConfigMotor1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_ladderMotor_2.configure(motorConfigMotor2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.putNumber("kLadderkP", kP);
    SmartDashboard.putNumber("kLadderkI", kI);
    SmartDashboard.putNumber("kLadderkD", kD);
    SmartDashboard.putNumber("kLadderkF", kF);
    SmartDashboard.putBoolean("New PIDF", false);


  }

  public void moveToHeight(double position){
    m_targetPosition = position;
  }
  public double getHeight(){
    return m_ladderMotor_1.getEncoder().getPosition();
  }
  public double getTargetHeight(){
    return m_targetPosition;
  }
  @Override
  
  public void periodic() {
    SmartDashboard.putNumber("Ladder Motor Output", m_ladderMotor_1.getOutputCurrent());

    SmartDashboard.putNumber("Lift Height", m_ladderMotor_1.getEncoder().getPosition());
    SmartDashboard.putNumber("Lift Velocity", m_ladderMotor_1.getEncoder().getVelocity());
    if(Constants.DriveConstants.enableLadder & disabled == false){
        if(m_targetPosition == Constants.LadderConstants.zeroPosition){
          m_ladderMotor_1.stopMotor();
          m_ladderMotor_2.stopMotor();
        }else{
          closedLoopController.setReference(m_targetPosition, ControlType.kPosition);
        }
    }else{
      m_ladderMotor_1.stopMotor();
      m_ladderMotor_2.stopMotor();
    }
  }
}
