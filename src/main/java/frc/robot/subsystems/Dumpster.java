// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dumpster extends SubsystemBase {
  private SparkMax m_dumpsterDown;

  private double dumpsterSpeed = SubsystemConstants.kDumpsterFast;

  public Dumpster() {
    m_dumpsterDown =  new SparkMax(Constants.SubsystemConstants.kDumpsterId, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    m_dumpsterDown.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  public void runDumpster(double speed) {
    m_dumpsterDown.set(dumpsterSpeed*speed);
  }

  public void slowMode(boolean isSlow) {
    if (isSlow) {
      dumpsterSpeed = SubsystemConstants.kDumpsterSlow;
    } else {
      dumpsterSpeed = SubsystemConstants.kDumpsterFast;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}