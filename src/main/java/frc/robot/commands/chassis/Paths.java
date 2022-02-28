package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.Arrays;

import com.ma5951.utils.autonomous.Path;
import com.ma5951.utils.autonomous.Waypoint;


public class Paths {

public static final Path getingOutOfLunchPad = new Path(
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2.3)), 0.2, 1.5, 3, 1, 0.3, 0.01);
}
