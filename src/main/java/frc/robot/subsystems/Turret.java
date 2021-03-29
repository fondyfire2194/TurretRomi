// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private static final double kCountsPerRevolution = 12.0;
  private static final double k_gearRatio = 120;// 360 degrees
  private static final double k_encoderCountsPerDegree = 360 / (kCountsPerRevolution * k_gearRatio);//
  private static final double k_cwLimit = 110;
  private static final double k_ccwLimit = -125;

  private final Spark m_turretMotor = new Spark(0);
  private final Encoder m_turretEncoder = new Encoder(4, 5);

  public Turret() {
    m_turretEncoder.setDistancePerPulse(k_encoderCountsPerDegree);
    m_turretEncoder.reset();
    SmartDashboard.putNumber("TurretTarget", 0);
  }

  public void turnTurret(double speed) {

    if (onCWLimit() && speed > 0)
      speed = 0;
    if (onCCWLimit() && speed < 0)
      speed = 0;
    m_turretMotor.set(speed);
  }

  public double getTurretPosition() {
    return m_turretEncoder.getDistance();
  }

  public void resetTurretPosition() {
    m_turretEncoder.reset();
  }

  public double getTurretRate() {

    return m_turretEncoder.getRate();
  }

  public boolean onCWLimit() {
    return getTurretPosition() >= k_cwLimit;
  }

  public boolean onCCWLimit() {
    return getTurretPosition() <= k_ccwLimit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TurretPosition", getTurretPosition());
    SmartDashboard.putNumber("TurretRae", getTurretRate());
    SmartDashboard.putBoolean("TurretCWLimit", onCWLimit());
    SmartDashboard.putBoolean("TurretCCWLimit", onCCWLimit());

  }

}
