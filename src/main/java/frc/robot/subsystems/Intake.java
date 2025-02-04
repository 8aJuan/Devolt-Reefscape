package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanIds;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  
  //motores de intake
  private SparkMax intake1 = new SparkMax(CanIds.kIntake1CanId,MotorType.kBrushed);
  public Intake() {
    //configurar sparks
    intake1.configure(Configs.Intake.intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setDefaultCommand(new RunCommand(()->{setMotor(0);}, this));
  }
  @Override
  public void periodic() {
  }
  public void setMotor(double speed){
    intake1.set(speed);
  }
}