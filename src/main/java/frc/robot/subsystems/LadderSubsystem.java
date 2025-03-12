// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
public class LadderSubsystem extends SubsystemBase {
  private static double kDt = 0.02;
  private static double kMaxVelocity = 0.5;
  private static double kMaxAcceleration = 0.25;
  private static double kP = 1.3;
  private static double kI = 0.0;
  private static double kD = 0.7;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;

  private final WPI_TalonSRX m_ladderMotor1 = new WPI_TalonSRX(OIConstants.ladderMotorPort1);
  private final WPI_TalonSRX m_ladderMotor2 = new WPI_TalonSRX(OIConstants.ladderMotorPort2);
  private final Encoder m_encoder = new Encoder(0, 1);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);


  public LadderSubsystem(LadderSubsystem subsystem){
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    m_ladderMotor2.follow(m_ladderMotor1);
  }

  public void moveToHeight(double position){
    m_controller.setGoal(position);
    m_ladderMotor1.setVoltage(
      m_controller.calculate(m_encoder.getDistance())
      + m_feedforward.calculate(m_controller.getSetpoint().velocity));
  }
 
  @Override

  public void periodic() {
    //System.out.println(m_encoder.getDistance());
    // This method will be called once per scheduler run
  }
}
