package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  
  private SparkMax wristSpark = new SparkMax(Constants.CanIds.wristCanId, MotorType.kBrushless);

  PIDController pid = new PIDController(.01, 0, 0);
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
    SmartDashboard.putNumber("wrist", wristSpark.getEncoder().getPosition());
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
    if (output > .5){
      wristSpark.set(.5);
    }else if(output < -.5){
      wristSpark.set(-.5);
    }else{
      wristSpark.set(output);
    }
    pid.close();
  }
  
  public double getControllerError() {
    return pid.getError();
  }
}
