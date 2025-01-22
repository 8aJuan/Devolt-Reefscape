package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  
  private SparkMax wristSpark = new SparkMax(Constants.CanIds.wristCanId, MotorType.kBrushless);
 
  private double kp = .05;

  public Wrist() {
    wristSpark.configure(Configs.Wrist.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristSpark.getEncoder().setPosition(0);
  }

  public void moveTo(double obj){
    double pos = wristSpark.getEncoder().getPosition();
    double error = obj - pos;
    double output = kp * error;
    if (output > .5){
      wristSpark.set(.5);
    }
    else if (output<-.5){
      wristSpark.set(-.5);
    }
    else{
    wristSpark.set(output);
    }
    //funcion para performar movimiento con pid
  }
  @Override
  public void periodic() {
    wristSpark.set(0);
    // This method will be called once per scheduler run
  }
  public void resetEncoder(){
    wristSpark.getEncoder().setPosition(0);
  }
}
