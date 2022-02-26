package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.autonomous.Path;
import frc.robot.autonomous.Waypoint;
import frc.robot.autonomous.Path.PathBuilder;

public class Paths {
    private static final PathBuilder builder = new Path.PathBuilder();

    // public static final Path getingOutOfLunchPad = builder // green & orange
    // .setLookaheadDistance(5)
    // .setMaxRate(0.05)
    // .setMaxAcceleration(0.33)
    // .setMaxVelocity(2.2)
    // .setSpacing(0.1)
    // .addPoint(0, 0, true)
    // .addPoint(0, -1)
    // .build();

    public static final Path getingOutOfLunchPad = new Path(
            Arrays.asList(new Waypoint(0, 0, true), new Waypoint(0, -1.75)), 0.1, 1.5, 3.7, 1, 0.2, 0.01);

    public static final Path turnToSecondBall = builder // blue
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

}
