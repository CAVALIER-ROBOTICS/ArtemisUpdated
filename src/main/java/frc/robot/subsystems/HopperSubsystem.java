// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax hopperFloor = new CANSparkMax(Constants.floorID, MotorType.kBrushless);
  CANSparkMax hopperWall = new CANSparkMax(Constants.wallID, MotorType.kBrushless);
  
  public HopperSubsystem() 
  {
    hopperWall.follow(hopperFloor);
    hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65400);
    hopperWall.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65300);
    hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65200);
    hopperWall.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65100);
  }

  public void setMotors(double x){
    hopperFloor.set(x);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
