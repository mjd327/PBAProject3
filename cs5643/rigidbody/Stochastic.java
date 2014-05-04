package cs5643.rigidbody;

import java.util.ArrayList;
import javax.media.opengl.GL2;
import javax.vecmath.*;

public class Stochastic
{
    /** 
     * A list of points showing the path of the object after the impulse is applied.
     * This should be populated when this Stochastic object is constructed.
     */
    public ArrayList<ArrayList<Point2d>> paths = new ArrayList<ArrayList<Point2d>>();

    public Stochastic()
    {
    	
    }
    /** 
     * Displays the object to the screen, given a color.
     * By default, draws the path as line segments.
     */
    public void display(GL2 gl, Color3f color) {
	gl.glColor3f(color.x, color.y, color.z);

	gl.glBegin(GL2.GL_LINES);
	for(ArrayList<Point2d> path : paths)
	{
		for (int i=0; i<path.size()-1; i++) {
			gl.glVertex2d(path.get(i).x, path.get(i).y);
		}
	}
	gl.glEnd();
	
    }

    /** Returns true if a given point intersects the path of the Stochastic object. Useful for selecting paths. */
    public boolean intersectsPoint(Point2d p, double tolerance) {
    	for(ArrayList<Point2d> path : paths)
    	{
    		for (int i=0; i<path.size()-1; i++) {
    			if (Utils.distPointToSegment(p, path.get(i), path.get(i+1)) < tolerance) return true;
    		}
    	}
	return false;
    }

}