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
  private SparkMax intake1 = new SparkMax(CanIds.kIntake1CanId,MotorType.kBrushless);
  private SparkMax intake2 = new SparkMax(CanIds.kIntake2CanId, MotorType.kBrushless);

  public Intake() {
    //configurar sparks
    intake1.configure(Configs.Intake.intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake2.configure(Configs.Intake.intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(new RunCommand(()->{setMotors(0, 0);}, this));
  }
  @Override
  public void periodic() {
  }
  public void setMotors(double speed1, double speed2){
    intake1.set(speed1);
    intake2.set(-speed2);
  }
}