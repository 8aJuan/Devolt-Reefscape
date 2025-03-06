// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.singlemotion.moveArm;
import frc.robot.Commands.singlemotion.moveElevator;

public class Base extends SubsystemBase {
  /** Creates a new Base. */
  public final Arm m_arm = new Arm(); 
  //public final Wrist m_wrist = new Wrist();
  public final Elevator m_elevator = new Elevator();
  public final Intake m_intake = new Intake();
  Timer timer = new Timer();
  private double armGrab = 65;
  private double armTarget = 0;
  private double elevTarget = 0;
  private int level = 1; //variable para cambiar velocidad de ruedas del intake

  public Base() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("level", level);
  }
  
  public void moveTo(){  // mover hacia el objetivo de cada mecanismo, encontrado en comando default en robot container
    this.movetoPosition(elevTarget, armTarget);
  }
  
  public void movetoPosition(double Elevator, double Arm){
    m_arm.moveTo(Arm);
    m_elevator.moveTo(Elevator);
  }
  
  public Command idlePositionCmd(Base m_base){ //guardar a la derecha
      level = 0;
      return new ParallelCommandGroup(
      new moveArm(m_arm, 0),
      new moveElevator(m_elevator, 0)
      );
    }
  
  public Command humanPositionCmd(Base m_base){
    level = 0;
    return new ParallelCommandGroup(
      new moveArm(m_arm, 10),
      new moveElevator(m_elevator, 6)
    );
  }

  public Command grabPositionCmd(Base m_base){
    return new ParallelCommandGroup(
      new moveArm(m_arm, armGrab),
      new moveElevator(m_elevator, 0)
      );
  }

  public Command scoreLv1(Base m_base){
    level = 1;
    return new ParallelCommandGroup(
      new moveArm(m_arm, 40),
      new moveElevator(m_elevator, .1)
    );
  }

  public Command scoreLv2(Base m_base){
    level = 2;
      return new ParallelCommandGroup(
        new moveArm(m_arm, 39), 
        new moveElevator(m_elevator, 6)
      );
  }
  
  public Command scoreLv3(Base m_base){
    level = 2;
      return new ParallelCommandGroup(
        new moveArm(m_arm, 39),
        new moveElevator(m_elevator, 21)
        );
  }

  public Command scoreLv4(Base m_base){
    level = 3;
      return new ParallelCommandGroup(
        new moveArm(m_arm, 45), 
        new moveElevator(m_elevator, 43) 
        );
  }

  public Command algaeLv1(Base m_base){
    level = 4;
      return new ParallelCommandGroup(
        new moveArm(m_arm, 45), 
        new moveElevator(m_elevator, 35) 
        );
  }



  public void grab(){
    m_intake.setMotors(-.7, -.7);
  }
  public void release(){
    //Speed 1 para rueda superior
    switch (level) {
      case 1:
        m_intake.setMotors(.5 , 1);
      break;
      case 2:
        m_intake.setMotors(1 , .9);
      break;
      case 3:
        m_intake.setMotors(1 , .5);
      break;
      default:
      m_intake.setMotors(1, 1);
      break;
    }
  }

  public void intakeOff(){
    m_intake.setMotors(0, 0);
  }
}
