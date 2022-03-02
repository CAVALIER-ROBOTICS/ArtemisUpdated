// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperFloorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax hopperFloor = new CANSparkMax(Constants.floorID, MotorType.kBrushless);
  
  public HopperFloorSubsystem() 
  {
    hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65400);
    hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65200);
  }

  public void setFloor(double x){
    hopperFloor.set(x);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
