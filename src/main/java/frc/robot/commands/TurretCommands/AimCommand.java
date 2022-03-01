// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Limelight;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimCommand extends PIDCommand {
  /** Creates a new AimCommand. */
  public AimCommand(TurretSubsystem turretSub) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0.025 , 0),
        // This should return the measurement
        () -> Limelight.getX(),
        // This should return the setpoint (can al6o be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // if(turretSub.isTurning()) {
          //   turretSub.turn();
          // }
          // else {
          //   turretSub.setTurret(output);
          // }
          turretSub.aim(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(turretSub);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
