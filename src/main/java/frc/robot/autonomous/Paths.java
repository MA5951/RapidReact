// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.Arrays;

import frc.robot.autonomous.Path.PathBuilder;

/**
 * Add your docs here.
 */
public class Paths {
    public static PathBuilder builder = new Path.PathBuilder();
    
    public static final Path getingOutOfLunchPad = new Path(
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 1.75)), 0.1, 1.5, 0.8, 0.2, 0.2, 0.03);  

}
