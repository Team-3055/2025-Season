// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;

@Logged
public class Dealgaefier extends SubsystemBase {
  public WPI_TalonSRX m_algaeMotor = new WPI_TalonSRX(10);//OIConstants.deAlgifierPort);
  private double motorSpeed = OIConstants.intakeSpeed;
  
  /** Creates a new ExampleSubsystem. */
  public Dealgaefier() {
    m_algaeMotor.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void algaeIn(){
    m_algaeMotor.set(motorSpeed);
  }
  public void algaeOut(){
    m_algaeMotor.set(-motorSpeed);
  }
  public void stop(){
    m_algaeMotor.stopMotor();
  }
}
