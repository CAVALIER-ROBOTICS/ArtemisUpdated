// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperWallSubsystem extends SubsystemBase {
  /** Creates a new HopperWallSubsystem. */
  CANSparkMax hopperWall = new CANSparkMax(Constants.wallID, MotorType.kBrushless);
  

  public HopperWallSubsystem() {
    hopperWall.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65300);
    hopperWall.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65100);
  }

  public void setWall(double x){
    hopperWall.set(x);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
