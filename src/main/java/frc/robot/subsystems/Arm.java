package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private SparkMax armSpark = new SparkMax(Constants.CanIds.armCanId, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
    armSpark.configure(Configs.Arm.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armSpark.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm", armSpark.getEncoder().getPosition());
  }

  public void resetEncoder(){
    armSpark.getEncoder().setPosition(0);
  }

  public void setMotor(double speed){
    armSpark.set(speed);
  }
  public double getEncoder(){
    return armSpark.getEncoder().getPosition();
  }

  public void moveTo(double target){
    PIDController pid = new PIDController(.02, 0, 0);
    pid.setSetpoint(target);
    
    double output = pid.calculate(armSpark.getEncoder().getPosition());
    if (output > .5){
      armSpark.set(.5);
    }else if(output < -.5){
      armSpark.set(-.5);
    }else{
      armSpark.set(output);
    }
    pid.close();
  }
}
