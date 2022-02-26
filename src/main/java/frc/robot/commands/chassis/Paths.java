package frc.robot.commands.chassis;

import com.ma5951.utils.autonomous.Path;
import com.ma5951.utils.autonomous.Path.PathBuilder;

public class Paths {
    private static final PathBuilder builder = new Path.PathBuilder();

    public static final Path greenPath = builder
                                            .setLookaheadDistance(5)
                                            .setMaxRate(0.05)
                                            .setMaxAcceleration(0.33)
                                            .setMaxVelocity(2.2)
                                            .setSpacing(0.1)
                                            .addPoint(0, -1, true)
                                            .build();

    public static final Path CHECKING_PATH = builder
                                                .setLookaheadDistance(5)
                                                .setMaxRate(0.05)
                                                .setMaxAcceleration(0.33)
                                                .setSpacing(0.1)
                                                .addPoint(0, 1)
                                                .addPoint(1, 2)
                                                .addPoint(1, 0)
                                                .build();
    
}
