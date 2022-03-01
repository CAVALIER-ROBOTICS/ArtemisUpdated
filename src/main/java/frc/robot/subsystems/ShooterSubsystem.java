// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
  CANSparkMax rightShooter = new CANSparkMax(Constants.rightShooterID,MotorType.kBrushless);
  CANSparkMax leftShooter = new CANSparkMax(Constants.leftShootID,MotorType.kBrushless);

  RelativeEncoder leftEnc = leftShooter.getEncoder();
  RelativeEncoder rightEnc = rightShooter.getEncoder();
  
  SparkMaxPIDController leftPID = leftShooter.getPIDController();
  SparkMaxPIDController rightPID = rightShooter.getPIDController();

  

  // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.4402, .13453, .0093554);



  public ShooterSubsystem() 
  {
    leftPID.setP(0.0002);
    leftPID.setI(0.0000025);
    leftPID.setD(0.0001);
    leftPID.setFF(.00027);

    rightPID.setP(0.0002);
    rightPID.setI(0.0000025);
    rightPID.setD(0.0001);
    rightPID.setFF(.00027);

    // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 64500);
    // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64400);

    // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64300);

  }

  /**
   * 
   * @param a Velocity to be set
   */
  public void setShooterVelocity(double rpm)
  {
    leftPID.setReference(rpm, ControlType.kVelocity);
    rightPID.setReference(rpm, ControlType.kVelocity);
    // leftShooter.set(1000, ControlType.kVelocity);
    // leftShooter.set(a);
    // rightShooter.set(a);
    // SmartDashboard.putNumber("Fly Wheel", getVolicty());
  }

  public void setShooter(double volt) {
    leftShooter.set(volt);
    rightShooter.set(volt);
  }

  public double getVolicty()
  {
    return (leftEnc.getVelocity()+rightEnc.getVelocity())/2;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler rn
    SmartDashboard.putNumber("Fly Wheel", getVolicty());
  }
}
