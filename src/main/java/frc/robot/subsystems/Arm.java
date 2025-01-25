// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private SparkMax armSpark = new SparkMax(Constants.CanIds.armCanId, MotorType.kBrushless);
  private double kp = .02;
  /** Creates a new Arm. */
  public Arm() {
    armSpark.configure(Configs.Arm.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armSpark.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm position", armSpark.getEncoder().getPosition());
  }
  public void moveTo(double obj){
    double pos = armSpark.getEncoder().getPosition();
    double error = obj - pos;
    double output = kp * error;
    if (output > .5){
      armSpark.set(.5);
    }
    else if (output<-.5){
      armSpark.set(-.5);
    }
    else{
    armSpark.set(output);
    }
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
}
