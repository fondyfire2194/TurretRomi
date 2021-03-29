// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class PositionTurret extends CommandBase {
  /** Creates a new PositionTurret. */

  private final Turret m_turret;
  private double m_position;

  private PIDController m_controller = new PIDController(0.003, 0, 0);

  public PositionTurret(Turret turret, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_position = position;
  
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    
    double output = m_controller.calculate(m_turret.getTurretPosition(), m_position);
    

   
    m_turret.turnTurret(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.turnTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_position - m_turret.getTurretPosition()) < 2 || m_turret.onCCWLimit() || m_turret.onCWLimit();
  }
}
