// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  CANSparkMax turret = new CANSparkMax(Constants.turretID, MotorType.kBrushless);
  RelativeEncoder encoder = turret.getEncoder();
  public int acceptedVolts = 30;

  boolean turningR;

  boolean turningL;
  
  public TurretSubsystem() 
  {
    turningR = false;
    turningL = false;
    turret.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64200);
  }

  public double getPos()
  {
    return encoder.getPosition();
  }

  public double getVolt()
  {
    return turret.getOutputCurrent();
  }

  public void setTurret(double volt)
  {
    turret.set(volt);
    // SmartDashboard.putNumber("volts", volt);

  }

  public void turn()
  {
    if(turningL) {
      setTurret(-.2);
    }

    if(turningR) {
      setTurret(0.2);
    }

    if(getPos()>34) {
      turningR = false;
    }

    if(getPos()<8) {
      turningL = false;
    }
  }

  public void updateEnc()
  {
    encoder.setPosition(0);
  }

  public boolean isTurning()
  {
    return turningL || turningR;
  }

  // public boolean isTurningRight()
  // {
  //   return getPos()<3;
  // }


  public void aim(double temp)
  {
    // setTurret(x);

    // if(getPos()<3) {
    //   turningR = true;
    // }

    // if(getPos()>39)
    // {
    //   turningL = true;
    // }

    double x = temp;
    
    if(!turningR && !turningL)
    {
      setTurret(x);
      x=0;
    }

    x=0;

    
    if(getPos()<1)
    {
      turningR = true;
      x=0;
    }
    
    else if(getPos()>41)
    {
      turningL = true;
      x=0;
    }

    if(getPos()>32 && turningR)
    {
      turningR = false;
      x=0;
    }

    if(getPos()<10 && turningL)
    {
      turningL = false;
      x=0;
    }

    if(turningL)
    {
      x=0;
      setTurret(-.2);
    }
    else if(turningR)
    {
      x=0;
      setTurret(0.2);
    }
    // SmartDashboard.putNumber("PID volt", x);
    // SmartDashboard.putBoolean("turning left", turningL);
    // SmartDashboard.putBoolean("turning right", turningR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Enc", encoder.getPosition());
    // SmartDashboard.putBoolean("right", right);
    // SmartDashboard.putBoolean("Left", left);
  }
}
