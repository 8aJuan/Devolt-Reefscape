// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.humanposition;
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
 
  //posiciones de wrist
  private double algaeGrab = -115;
  private double coralGrab = 115;
  
  private double armTarget = 0;
  private double wristTarget = 0;
  private double elevTarget = 0;

  public Base() {
  }

  @Override
  public void periodic() {
    moveTo();
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
    wristTarget = isAlgae ? algaeGrab : coralGrab;
    sequential = false;
  }
  public void humanPosition(){
    new SequentialCommandGroup(
      new moveArm(m_arm, 15),
      new ParallelCommandGroup(new moveWrist(m_wrist, coralGrab), new moveElevator(m_elevator, 0)),
      new moveArm(m_arm, 3)
    );
    armTarget = 15;
    wristTarget = 115;
    elevTarget = 0;
    sequential = true;
  }
  public void Score(int level, Boolean RightSide){ //TODO introducir alturas de elevador 
    double sidePosition = RightSide ? 0 : -180;
    if (isAlgae){ //posiciones para manipular alga
      switch (level) {
        case 1: //anotar algae en processor
          grabPosition();
          break;
        case 2: // agarrar algae en posicion baja de reef
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0), 
            new ParallelCommandGroup(new moveArm(m_arm, 45), new moveWrist(m_wrist, algaeGrab))
            );
          break;
          case 3:// agarrar algae en posicion alta de reef
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0), 
            new ParallelCommandGroup(new moveArm(m_arm, 45), new moveWrist(m_wrist, algaeGrab))
           );
          break;
          case 4: // anotar algae en net
            new ParallelCommandGroup(
              new moveArm(m_arm, 0), 
              new moveWrist(m_wrist, 0),
              new moveElevator(m_elevator, 0));
      
        default:
          break;
      }
    }else{ 
      switch(level){
        case 1: // anotar en nivel 1 de reef
          new humanposition();
        break;
        case 2: // anotar en nivel 2 del lado que se introduzca
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0),
            new ParallelCommandGroup(new moveArm(m_arm, 45), 
            new moveWrist(m_wrist, sidePosition))
          );
        break;
        case 3: // anotar en nivel 3 del lado que se introduzca
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0),
            new ParallelCommandGroup(new moveArm(m_arm, 45), 
            new moveWrist(m_wrist, sidePosition))
          );
        break;
        case 4: // anotar en nivel 4 del lado que se introduzca
          new SequentialCommandGroup(
            new moveElevator(m_elevator, 0),
            new ParallelCommandGroup(new moveArm(m_arm, 45), 
            new moveWrist(m_wrist, sidePosition))
          );
        break;
      }
    }

  }

  public void Grab(){
    m_intake.setMotor(isAlgae ? -.5 : .5);
  }
  public void release(){
    m_intake.setMotor(isAlgae ? .5 : -.5);
  }
}
