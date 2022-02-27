package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.autonomous.Path;
import frc.robot.autonomous.Waypoint;
import frc.robot.autonomous.Path.PathBuilder;

public class Paths {

public static final Path getingOutOfLunchPad = new Path(
        Arrays.asList(new Waypoint(0, 0, true), new Waypoint(0, -1.75)), 0.1, 1.5, 3.7, 1, 0.2, 0.01);  
}
