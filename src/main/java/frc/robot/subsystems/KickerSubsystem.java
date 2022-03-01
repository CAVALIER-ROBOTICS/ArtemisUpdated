// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
  CANSparkMax kickerMotor = new CANSparkMax(Constants.kickerID,MotorType.kBrushless);

  public KickerSubsystem() 
  {
    kickerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 64700);
    kickerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64600);
  }

  public void setKicker(double x)
  {
    kickerMotor.set(x);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
