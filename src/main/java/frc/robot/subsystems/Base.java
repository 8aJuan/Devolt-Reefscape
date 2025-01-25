// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.singlemotion.moveArm;
import frc.robot.Commands.singlemotion.moveElevator;
import frc.robot.Commands.singlemotion.moveWrist;

public class Base extends SubsystemBase {
  /** Creates a new Base. */
  private final Arm m_arm = new Arm(); 
  private final Wrist m_wrist = new Wrist();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();

  private Boolean isAlgae = false; // variable para cambiar a modo de cada pieza
  private Boolean sequential = false; //determina se se requiere movimiento secuencial despues de la posicion
 
  private double armTarget = 0;
  private double wristTarget = 0;
  private double elevTarget = 0;

  public Base() {
  }

  @Override
  public void periodic() {

  }
  public void moveTo(){  // mover hacia el objetivo de cada mecanismo, encontrado en comando default en robot container
    new moveArm(m_arm, armTarget);
    new moveElevator(m_elevator, elevTarget);
    new moveWrist(m_wrist, wristTarget);

  }
  public void toggleGamePiece (){  // cambiar pieza
    isAlgae = isAlgae ? false : true;
    
  }
  public void idlePosition(){
    if (sequential){ //movimientos secuenciales para evitar colision
      new SequentialCommandGroup(
        new moveArm(m_arm, 15),
        new ParallelCommandGroup(new moveElevator(m_elevator, 0), new moveWrist(m_wrist, 0)),
        new moveArm(m_arm, 0)
      );
    }
    armTarget = 0;
    wristTarget = 0;
    elevTarget = 0;
    sequential = false;
  }
  public void grabPosition(){
    armTarget = 85;
    elevTarget = 0;
    wristTarget = isAlgae ? -115 : 115;
    sequential = false;
  }
  public void humanPosition(){
    new SequentialCommandGroup(
      new moveArm(m_arm, 15),
      new ParallelCommandGroup(new moveWrist(m_wrist, 115), new moveElevator(m_elevator, 0)),
      new moveArm(m_arm, 3)
    );
    armTarget = 15;
    wristTarget = 115;
    elevTarget = 0;
    sequential = true;
  }
  public void Score(int level, Boolean RightSide){
    double wrist = RightSide ? 0 : -180;
    if (isAlgae){ //posiciones para manipular alga
      switch (level) {
        case 1:
          grabPosition();
          break;
        case 2:
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0), 
            new ParallelCommandGroup(new moveArm(m_arm, 45), new moveWrist(m_wrist, wrist))
            );
          break;
          case 3:
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0), 
            new ParallelCommandGroup(new moveArm(m_arm, 45), new moveWrist(m_wrist, wrist))
           );
          break;
          case 4:
            new ParallelCommandGroup(
              new moveArm(m_arm, 0), 
              new moveWrist(m_wrist, 0),
              new moveElevator(m_elevator, 0));
      
        default:
          break;
      }
    }else{ //TODO posiciones para anotar coral

    }

  }

  public void Grab(){
    m_intake.setMotor(isAlgae ? -.5 : .5);
  }
  public void release(){
    m_intake.setMotor(isAlgae ? .5 : -.5);
  }
}
