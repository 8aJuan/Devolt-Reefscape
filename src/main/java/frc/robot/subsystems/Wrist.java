package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  
  private SparkMax wristSpark = new SparkMax(Constants.CanIds.wristCanId, MotorType.kBrushless);

  PIDController pid = new PIDController(.017, 0, .0);
  double lastTargetPosition;
  
  public Wrist() {
    wristSpark.configure(Configs.Wrist.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristSpark.getEncoder().setPosition(0);

    setDefaultCommand(new RunCommand(() -> {
      moveTo(lastTargetPosition);
    }, this));
  }

  @Override
  public void periodic() {
  }
  public void resetEncoder(){
    wristSpark.getEncoder().setPosition(0);
  }
  public void setMotor(double speed){
    wristSpark.set(speed);
  }
  public double getEncoder(){
    return wristSpark.getEncoder().getPosition();
  }

  public void moveTo(double target){
    lastTargetPosition = target;
    pid.setSetpoint(target);
    
    double output = pid.calculate(wristSpark.getEncoder().getPosition());
    if (output > .2){
      wristSpark.set(.2);
    }else if(output < -.2){
      wristSpark.set(-.2);
    }else{
      wristSpark.set(output);
    }
    pid.close();
  }
  
  public double getControllerError() {
    return pid.getError();
  }
}
