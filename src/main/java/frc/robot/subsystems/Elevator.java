package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax elev1 = new SparkMax(Constants.CanIds.elev1CanId, MotorType.kBrushless);
  private SparkMax elev2 = new SparkMax(Constants.CanIds.elev2CanId, MotorType.kBrushless);


  public Elevator() {
    elev1.configure(Configs.Elevator.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elev2.configure(Configs.Elevator.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator", elev1.getEncoder().getPosition());
  }

  public void setMotors(double speed){
    elev1.set(speed);
    elev2.set(-speed);
  }

  public double getEncoder(){
    return elev1.getEncoder().getPosition();
  }

  public void resetEncoder(){
    elev1.getEncoder().setPosition(0);
  }
}
