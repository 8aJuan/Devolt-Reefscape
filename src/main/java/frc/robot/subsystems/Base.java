// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.singlemotion.moveArm;
import frc.robot.Commands.singlemotion.moveElevator;
import frc.robot.Commands.singlemotion.moveWrist;

public class Base extends SubsystemBase {
  /** Creates a new Base. */
  public final Arm m_arm = new Arm(); 
  public final Wrist m_wrist = new Wrist();
  public final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();

  private Boolean isAlgae = false; // variable para cambiar a modo de cada pieza
  private Boolean sequential = false; //determina se se requiere movimiento secuencial despues de la posicion

  //posiciones de wrist
  private double algaeGrab = 115;
  private double coralGrab = -115;
  private double right = 0;
  private double left = 200;

  //posiciones Arm
  private double armScore = 20;
  private double armseq = 15;
  private double zero = 0;
  private double armGrab = 85;
  
  private double armTarget = 0;
  private double wristTarget = 0;
  private double elevTarget = 0;

  public Base() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Algae", isAlgae);
    SmartDashboard.putBoolean("secuential", sequential);
  }
  
  public void moveTo(){  // mover hacia el objetivo de cada mecanismo, encontrado en comando default en robot container
    this.movetoPosition(elevTarget, armTarget, wristTarget);
  }
  
  public void movetoPosition(double Elevator, double Arm, double Wrist){
    m_arm.moveTo(Arm);
    m_elevator.moveTo(Elevator);
    m_wrist.moveTo(Wrist);
  }

  public void setSecuential(boolean seq){
    sequential = seq;
  }
  
  public void toggleGamePiece (){  // cambiar pieza
    isAlgae = isAlgae ? false : true;
    
  }

  public Command idlePositionCmd(Base m_base){
    if(sequential){
    return new SequentialCommandGroup(
      new moveArm(m_arm, 15),
      new moveWrist(m_wrist, right),
      new moveArm(m_arm, zero),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(false);})
    );
    }else{
      return new ParallelCommandGroup(
      new moveWrist(m_wrist, right),
      new moveArm(m_arm, 0),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(false);})
      );
    }
  }

  public Command humanPositionCmd(Base m_base){
    return new SequentialCommandGroup(
      new moveArm(m_arm, 15),
      new moveWrist(m_wrist, coralGrab),
      new moveArm(m_arm, 0),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
  }

  public Command grabPositionCmd(Base m_base){
    return new ParallelCommandGroup(
      new moveWrist(m_wrist, isAlgae ? algaeGrab : coralGrab),
      new moveArm(m_arm, armGrab),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(false);})
      );
  }

  public Command scoreLv1(Base m_base){
    if (isAlgae){
      return new ParallelCommandGroup(
        new moveWrist(m_wrist, isAlgae ? algaeGrab : coralGrab),
        new moveArm(m_arm, 85),
        new moveElevator(m_elevator, 0),
        new InstantCommand(()->{m_base.setSecuential(false);})
        );
    }
    return new SequentialCommandGroup(
      new moveArm(m_arm, 15),
      new moveWrist(m_wrist, coralGrab),
      new moveArm(m_arm, 0),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
  }

  public Command scoreLv2(Boolean rightSide, Base m_base){
    if (isAlgae){
      return new SequentialCommandGroup(
        new moveElevator(m_elevator, 20), 
        new ParallelCommandGroup(new moveArm(m_arm, 70), new moveWrist(m_wrist, algaeGrab)),
        new InstantCommand(()->{m_base.setSecuential(true);})
        );
    }else{
    return new SequentialCommandGroup(
      new moveElevator(m_elevator, 7),
      new ParallelCommandGroup(new moveArm(m_arm, armScore), 
      new moveWrist(m_wrist, rightSide ? right : left)),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
    }
  }
  
  public Command scoreLv3(Boolean rightSide, Base m_base){
    if (isAlgae){
      return new SequentialCommandGroup(
        new moveElevator(m_elevator, 25), 
        new ParallelCommandGroup(new moveArm(m_arm, 70), new moveWrist(m_wrist, algaeGrab)),
        new InstantCommand(()->{m_base.setSecuential(true);})
        );
    }else{
    return new SequentialCommandGroup(
      new moveElevator(m_elevator, armseq),
      new ParallelCommandGroup(
      new moveArm(m_arm, armScore), 
      new moveWrist(m_wrist, rightSide ? right : left)),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
    }
  }

  public Command scoreLv4(Boolean rightSide, Base m_base){
    if (isAlgae){
      return new ParallelCommandGroup(
        new moveArm(m_arm, 15), 
        new moveWrist(m_wrist, right),
        new InstantCommand(()->{m_base.setSecuential(true);})
        );
    }else{
    return new SequentialCommandGroup(
      new moveElevator(m_elevator, 35),
      new ParallelCommandGroup(new moveArm(m_arm, armScore), 
      new moveWrist(m_wrist, rightSide ? right : left)),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
    }
  }

  public void grab(){
    m_intake.setMotor(isAlgae ? -.3 : .3);
  }
  
  public void release(){
    m_intake.setMotor(isAlgae ? .3 : -.3);
  }
  
  public void intakeoff(){
    m_intake.setMotor(0);
  }
}
