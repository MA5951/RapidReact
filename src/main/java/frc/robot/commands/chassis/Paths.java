package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.Arrays;

import com.ma5951.utils.autonomous.Path;
import com.ma5951.utils.autonomous.Waypoint;


public class Paths {

public static final Path getingOutOfLunchPadPart1 = new Path(
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 1.4)), 0.2, 1.5, 3.7, 2.55, 0.3, 0.02);

public static final Path getingOutOfLunchPadPart2 = new Path(
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 0.9)), 0.2, 1.5, 3, 1, 0.3, 0.01);

public static final Path goToTheSecondBallPart1 = new Path(
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 0.6),
         new Waypoint(-0.8, 0)), 0.2, 1.5, 1.5, 0.5, 0.3, 0.01);
public static final Path goToTheSecondBallPart2 = new Path(
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0.7, 2.2)), 0.2, 1.5, 1.75, 0.75, 0.3, 0.01);

public static final Path goToTheSecondBallPart3= new Path (
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2), new Waypoint (-0.5, 1.75))
        , 0.2, 1.5, 2, 1, 0.3, 0.01);

// public static final Path goToTheSecondBallPart4= new Path (
//         Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 0.2))
//         , 0.2, 1.5, 2, 1, 0.1, 0.01);

public static final Path goToHPBallPart1 = new Path (
        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 1.8), new Waypoint(0.8, 1))
        , 0.2, 1.5, 2, 1, 0.3, 0.01);



// public static final Path goToHPBallPart1 = new Path()

}
