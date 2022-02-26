package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.Arrays;

import com.ma5951.utils.autonomous.Path;
import com.ma5951.utils.autonomous.Waypoint;
import com.ma5951.utils.autonomous.Path.PathBuilder;

public class Paths {
    private static final PathBuilder builder = new Path.PathBuilder();

    public static final Path getingOutOfLunchPad = builder // green & orange
                                            .setLookaheadDistance(5)
                                            .setMaxRate(0.05)
                                            .setMaxAcceleration(0.33)
                                            .setMaxVelocity(2.2)
                                            .setSpacing(0.1)
                                            .addPoint(0, -1, true)
                                            .build();

    public static final Path turnToSecondBall = builder //blue
                                            .setLookaheadDistance(5)
                                            .setMaxRate(0.05)
                                            .setMaxAcceleration(0.33)
                                            .setMaxVelocity(2.2)
                                            .setSpacing(0.1)
                                            .addPoint(-1, -1, true)
                                            .build();

    public static final Path moveToHumanPlayer = builder // red
                                            .setLookaheadDistance(5)
                                            .setMaxRate(0.05)
                                            .setMaxAcceleration(0.33)
                                            .setMaxVelocity(2.2)
                                            .setSpacing(0.1)
                                            .addPoint(0, -2, true)
                                            .build();
    

    public static final Path moveAfterHumanPlayer = builder // purple
                                            .setLookaheadDistance(5)
                                            .setMaxRate(0.05)
                                            .setMaxAcceleration(0.33)
                                            .setMaxVelocity(2.2)
                                            .setSpacing(0.1)
                                            .addPoint(0, 1, false)
                                            .build();

    public static Path CHECKING_PATH = new Path( Arrays.asList(new Waypoint(0, 1))
                                            , 0, 0, 0, 0, 0, 0);
    
}
