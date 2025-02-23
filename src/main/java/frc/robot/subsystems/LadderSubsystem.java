package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class LadderSubsystem extends SubsystemBase {

  /*private final WPI_TalonSRX ladderMotor1 = new WPI_TalonSRX(OIConstants.ladderMotorPort1);
  private final WPI_TalonSRX ladderMotor2 = new WPI_TalonSRX(OIConstants.ladderMotorPort2);
  private double ladderMotorSpeed = OIConstants.ladderMotorSpeed;

  public LadderSubsystem(LadderSubsystem subsystem){
    ladderMotor2.follow(ladderMotor1);
  }

  public void moveUp() {
    ladderMotor1.set(ladderMotorSpeed);
  }

  public void moveDown() {
    ladderMotor1.set(-ladderMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }*/
}
