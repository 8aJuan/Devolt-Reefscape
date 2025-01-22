package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax elev1 = new SparkMax(Constants.CanIds.elev1CanId, MotorType.kBrushless);
  private SparkMax elev2 = new SparkMax(Constants.CanIds.elev2CanId, MotorType.kBrushless);
  private double kp = .5;

  public Elevator() {
    elev1.configure(Configs.Elevator.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elev2.configure(Configs.Elevator.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  
  public void moveTo(double height){

    double pos = elev1.getEncoder().getPosition();
    double error = height - pos;

    double output = kp * error;
    elev1.set(output);
    elev2.set(-output);

    //funcion para performar movimiento con pid
    
  }

}
