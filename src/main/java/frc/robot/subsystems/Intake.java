package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanIds;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  
  //motores de intake
  private SparkMax intake1 = new SparkMax(CanIds.kIntake1CanId,MotorType.kBrushless);
  public Intake() {
    //configurar sparks
    intake1.configure(Configs.Intake.intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    intake1.set(0);
    // This method will be called once per scheduler run
  }
  public void grab(){
    intake1.set(.5);
  }
  public void release(){
    intake1.set(-.5);
  }
}