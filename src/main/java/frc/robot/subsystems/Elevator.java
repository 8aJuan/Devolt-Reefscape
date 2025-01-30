package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax elev1 = new SparkMax(Constants.CanIds.elev1CanId, MotorType.kBrushless);
  private SparkMax elev2 = new SparkMax(Constants.CanIds.elev2CanId, MotorType.kBrushless);

  double lastTargetPosition;
  //diferentes pid para movimiento arriba y abajo
  PIDController upPid = new PIDController(.05, .004, 0);
  PIDController downPid = new PIDController(.02, 0, 0);


  public Elevator() {
    elev1.configure(Configs.Elevator.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elev2.configure(Configs.Elevator.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    setDefaultCommand(new InstantCommand(() -> {
      moveTo(lastTargetPosition);
    }, this));
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

  public void moveTo(double target){
    lastTargetPosition = target;
    upPid.setSetpoint(target);
    downPid.setSetpoint(target);
    
    if (target > elev1.getEncoder().getPosition()){ //arriba
      double output = upPid.calculate(elev1.getEncoder().getPosition());
    if (output > .5){
      elev1.set(.5);
      elev2.set(-.5);
    }else{
      elev1.set(output);
      elev2.set(-output);
    }
    }else{ //abajo
      double output = downPid.calculate(elev1.getEncoder().getPosition());
      if(output < -.4){
        elev1.set(-.4);
        elev2.set(.5);
      }
      else{
        elev1.set(output);
        elev2.set(-output);
      }
    }
    upPid.close();
    downPid.close();
  }

  public double getControllerError(){
    return upPid.getError();
  }
}
