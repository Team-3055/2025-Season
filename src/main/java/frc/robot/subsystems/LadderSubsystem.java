package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;

public class LadderSubsystem extends SubsystemBase {

  private final WPI_TalonSRX ladderMotor1 = new WPI_TalonSRX(OIConstants.ladderMotorPort1);
  private final WPI_TalonSRX ladderMotor2 = new WPI_TalonSRX(OIConstants.ladderMotorPort2);
  private double ladderMotorSpeed = OIConstants.ladderMotorSpeed;
  private Encoder m_encoder = new Encoder(0, 1);
  private ProfiledPIDController positionPID = new ProfiledPIDController(ladderMotorSpeed, ladderMotorSpeed, ladderMotorSpeed, null);
  private double stallSpeed = 0.01;

  public LadderSubsystem(){
    m_encoder.setDistancePerPulse(1);
    ladderMotor2.follow(ladderMotor1);
  }

  public void moveToHeight(double position){
    double ladder_distance = positionPID.calculate(m_encoder.getDistance(), position);
    double ladder_voltage = ladder_distance / m_encoder.getDistance();
    if(ladder_voltage > 1){
      ladder_voltage = 1;
    } else if(ladder_voltage < -1){
      ladder_voltage = -1;
    }
    ladderMotor1.set(ladder_voltage);
  }
  public void moveUp(double voltage) {
    ladderMotor1.setVoltage(voltage);
  }

  public void moveDown() {
    ladderMotor1.set(-ladderMotorSpeed);
  }

  public void stall(){
    ladderMotor1.set(stallSpeed);
    ladderMotor1.set(-stallSpeed);
  }

  public void stop(){
    ladderMotor1.stopMotor();

  }
  @Override

  public void periodic() {
    //System.out.println(m_encoder.getDistance());
    // This methofd will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
