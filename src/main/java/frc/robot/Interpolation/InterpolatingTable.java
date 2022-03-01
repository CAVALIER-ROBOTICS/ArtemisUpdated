// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

/** Add your docs here. */
public class InterpolatingTable 
{
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            entry(.9, new ShotParameter(23, 1950, 0.0)),
            entry(1.5, new ShotParameter(25, 2000, 0.0)),
            entry(1.8, new ShotParameter(25, 2000, 0.0)),
            entry(2.0, new ShotParameter(27, 2100, 0.0)),
            entry(2.4, new ShotParameter(27, 2300, 0.0)),
            entry(2.8, new ShotParameter(30, 2400, 0.0)),
            entry(3.2, new ShotParameter(30, 2500, 0.0)),
            entry(3.6, new ShotParameter(30, 2600, 0.0)),
            entry(4.0, new ShotParameter(38, 2600, 0.0)),
            entry(4.3, new ShotParameter(40, 2800, 0.0))
        )
    );

    public static ShotParameter getShotParameter(double distance)
    {
        Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
        Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
        if (ceilEntry == null)
        {
            return floorEntry.getValue();
        } 

        if (floorEntry == null)
        {
            return ceilEntry.getValue();
        }

        if (ceilEntry.getValue().equals(floorEntry.getValue()))
        {
            return ceilEntry.getValue();
        } 
        
        return ceilEntry.getValue().interpolate(
            floorEntry.getValue(), 
            (distance - floorEntry.getKey())/(ceilEntry.getKey() - floorEntry.getKey())
        );
    } 
}
