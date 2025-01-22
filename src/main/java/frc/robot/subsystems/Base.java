// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
  /** Creates a new Base. */
  private final Arm m_arm = new Arm(); 
  private final Wrist m_wrist = new Wrist();
  private final Elevator m_elevator = new Elevator();

  private String gamePiece = "coral"; // variable para cambiar a modo de cada pieza
 
  private double armTarget = 0;
  private double wristTarget = 0;
  private double elevTarget = 0;

  public Base() {
  }

  @Override
  public void periodic() {

  }
  public void moveTo(){  // mover hacia el objetivo de cada mecanismo, encontrado en comando default en robot container
    m_arm.moveTo(armTarget);
    m_wrist.moveTo(wristTarget);
    m_elevator.moveTo(elevTarget);
  }
  public void toggleGamePiece (){  // cambiar pieza
    gamePiece = gamePiece == "coral" ? "algae" : "coral";
    
  }
  public void idlePosition(){
    armTarget = 0;
    wristTarget = 0;
    elevTarget = 0;
  }
  public void grabPosition(){
    armTarget = 85;
    elevTarget = 0;
    wristTarget = gamePiece == "algae" ? -115 : 120;
  }
  public void humanPosition(){
    armTarget = 45;
    wristTarget = -105;
    elevTarget = 0;
  }
  public void Score(int level, Boolean RightSide){
    armTarget = 45;
    wristTarget = 0;
    elevTarget = 0;
  }

}
