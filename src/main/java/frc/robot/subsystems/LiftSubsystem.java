// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;

public class LiftSubsystem extends SubsystemBase {
  private WPI_TalonSRX liftActuator = new WPI_TalonSRX(OIConstants.liftMotorPort);
  
  /** Creates a new Lift subsystem. */
  public LiftSubsystem() {}
  
  public void feedForward(double speed){
    liftActuator.set(speed);
  }
  public void feedForward(){
    liftActuator.set(1);
  }
  public void stop(){
    liftActuator.set(0);
  }
  
  @Override
  public void periodic() {
    
  }
}
