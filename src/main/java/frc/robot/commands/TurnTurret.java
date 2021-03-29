// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurnTurret extends CommandBase {
  /** Creates a new TurnTurret. */
  private final Turret m_turret;
  private final XboxController m_controller;

  public TurnTurret(Turret turret, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_controller = controller;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_speed = -m_controller.getRawAxis(4);
    m_turret.turnTurret(m_speed/4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.turnTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
