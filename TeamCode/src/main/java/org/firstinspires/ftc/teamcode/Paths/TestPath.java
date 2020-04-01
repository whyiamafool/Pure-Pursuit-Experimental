package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.PurePursuit.MovementPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;

import java.util.ArrayList;

public class TestPath {

    private ArrayList<PurePursuit> paths;

    public TestPath() {
        paths = new ArrayList();

        ArrayList<MovementPoint> points = new ArrayList();

        points.add(new MovementPoint(50,50,0));
        paths.add(new PurePursuit(points));
    }

    public ArrayList<PurePursuit> getPaths() {
        return paths;
    }

}
