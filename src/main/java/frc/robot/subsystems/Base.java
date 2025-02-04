// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
  private double algaeGrab = -90;
  private double coralGrab = 100;
  private double right = 0;
  private double left = 200;

  //posiciones Arm
  private double armScore = 40;
  private double armseq = 15;
  private double zero = 0;
  private double armGrab = 88;
  
  private double armTarget = 0;
  private double wristTarget = 0;
  private double elevTarget = 0;

  public Base() {
  }

  @Override
  public void periodic() {
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

  public Command idlePositionCmd(Base m_base){ //guardar a la derecha
    if(sequential){
    return new SequentialCommandGroup(
      new moveArm(m_arm, armseq),
      new moveWrist(m_wrist, right),
      new moveArm(m_arm, zero),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
    }else{
      return new ParallelCommandGroup(
      new moveWrist(m_wrist, right),
      new moveArm(m_arm, 0),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(true);})
      );
    }
  }
  
  public Command idlePositionLeftCmd(Base m_base){ //guardar a la izquierda
    return new SequentialCommandGroup(
      new moveArm(m_arm, armseq),
      new moveWrist(m_wrist, left),
      new moveArm(m_arm, zero),
      new moveElevator(m_elevator, 0),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
  }

  public Command humanPositionCmd(Base m_base){
    return new SequentialCommandGroup(
      new moveArm(m_arm, 15),
      new moveWrist(m_wrist, coralGrab),
      new moveArm(m_arm, 0),
      new moveElevator(m_elevator, 5),
      new InstantCommand(()->{m_base.setSecuential(true);})
    );
  }

  public Command grabPositionCmd(Base m_base){
    return new ParallelCommandGroup(
      new moveWrist(m_wrist, isAlgae ? algaeGrab : coralGrab),
      new moveArm(m_arm, isAlgae ? armGrab + 3 : armGrab),
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
        new moveElevator(m_elevator, 15), 
        new ParallelCommandGroup(new moveArm(m_arm, 70), new moveWrist(m_wrist, algaeGrab)),
        new InstantCommand(()->{m_base.setSecuential(true);})
        );
    }else{
      return new SequentialCommandGroup(
        new InstantCommand(()->{m_base.setSecuential(true);}),
        new moveElevator(m_elevator, 6),
        new moveArm(m_arm, armScore),
        new ParallelCommandGroup(
          new moveElevator(m_elevator, 0),
          new InstantCommand(()->{m_base.grab();})
        ),
        new InstantCommand(()->{m_base.intakeOff();}),
        new moveArm(m_arm, zero)
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
        new InstantCommand(()->{m_base.setSecuential(true);}),
        new moveElevator(m_elevator, 20),
        new moveArm(m_arm, armScore),
        new ParallelCommandGroup(
          new moveElevator(m_elevator, 15),
          new InstantCommand(()->{m_base.grab();})
        ),
        new InstantCommand(()->{m_base.intakeOff();}),
        new moveArm(m_arm, zero)
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
      new moveElevator(m_elevator, 40),
      new ParallelCommandGroup(new moveArm(m_arm, armScore), 
      new InstantCommand(()->{m_base.setSecuential(true);})
    ));
    }
  }

  public void grab(){
    m_intake.setMotor(isAlgae ? .5 : -.5);
  }
  
  public void release(){
    m_intake.setMotor(isAlgae ? -.5 : .5);
  }
  
  public void intakeOff(){
    m_intake.setMotor(0);
  }
}
