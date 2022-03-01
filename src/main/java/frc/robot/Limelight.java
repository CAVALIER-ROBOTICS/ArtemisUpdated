// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interpolation.InterpolatingTable;

/** Add your docs here. */
public class Limelight {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-cavbots");

    public static NetworkTableEntry tx = table.getEntry("tx");
    public static NetworkTableEntry ty = table.getEntry("ty");
    public static NetworkTableEntry ta = table.getEntry("ta");

    public static double x;
    public static double y;
    public static double area;
    
    public Limelight() {
        // table = NetworkTableInstance.getDefault().getTable("limelight-cavbots");
        // tx = table.getEntry("tx");
        // ty = table.getEntry("ty");
        // ta = table.getEntry("ta");

        //read values periodically
        // x = tx.getDouble(0.0);
        // y = ty.getDouble(0.0);
        // area = ta.getDouble(0.0);
    }

    public static void updateValues() {
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        SmartDashboard.putNumber("lime Dist.", getDistance());
        // SmartDashboard.putNumber("X v",x);
        // SmartDashboard.putNumber("Y v", y);
        // SmartDashboard.putNumber("Area values", area);
    }

    public static double getX()
    {
        return x;
    }

    public static double getY()
    {
        return y;
    }

    public static double getArea()
    {
        return area;
    }

    public static double getDistance() {
        return ((2.74-.768) / (Math.tan(Math.toRadians(48+getY()))));
    }

    public static double getRPM()
    {
        return InterpolatingTable.getShotParameter(getDistance()).getRPM();
    }

    public static double getAngle()
    {
        return InterpolatingTable.getShotParameter(getDistance()).getHoodAngle();
    }
}
