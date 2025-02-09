package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class LadderSubsystem extends SubsystemBase {

  private int ladderHeight;
  private int minHeight;
  private int maxHeight;
  private final TalonFX ladderMotor = new TalonFX(Constants.OIConstants.ladderMotorPort);

  public LadderSubsystem(){
    this(0, 0, 12);
  }

  public LadderSubsystem(int initHeight, int mn_height, int mx_height) {
    ladderHeight = initHeight;
    minHeight = mn_height;
    maxHeight = mx_height;
    shiftToHeight();
  }

  public void setHeight(int height){
    if(height > maxHeight || height < minHeight){
      // do nothing
    } else {
      ladderHeight = height;
    } 
    shiftToHeight();
  }

  public void increaseHeight(){
    if(ladderHeight >= maxHeight){
      // do nothing
    } else {
      ladderHeight++;
    }
    shiftToHeight();
  }

  public void decreaseHeight(){
    if(ladderHeight <= minHeight){
      // do nothing
    } else {
      ladderHeight--;
    }    
    shiftToHeight();
  }

  public void shiftToHeight(){
    // implement motors to turn until ladder
    // is at desired height
    ladderMotor.set(ladderHeight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
