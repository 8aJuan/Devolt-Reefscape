package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Configs;

public class Climber extends SubsystemBase{

private SparkMax Climberspark = new SparkMax(Constants.CanIds.climberCanid, MotorType.kBrushless);

public Climber(){
    Climberspark.configure(Configs.Climber.climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(new RunCommand(()->{setMotor(0);}, this));

}
@Override
public void periodic() {
}
public void setMotor(double speed){
    Climberspark.set(speed);
  }

}