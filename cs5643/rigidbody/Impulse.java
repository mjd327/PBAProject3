package cs5643.rigidbody;

import java.util.Random;
import javax.media.opengl.GL2;
import javax.vecmath.*;

public class Impulse extends Stochastic {
    
    private static Random r = new Random();
    private RigidBody rb;
    private Vector2d force;

    public Impulse(RigidBody rb) {
	this.rb = rb;
	// set force
	// populate path

	// Testing ...
	// path.add(new Point2d(0, 0));
	// path.add(new Point2d(.2, .5));
	// path.add(new Point2d(1, 1));
    }

}