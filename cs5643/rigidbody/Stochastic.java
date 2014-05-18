package cs5643.rigidbody;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Iterator;

import javax.media.opengl.GL2;
import javax.vecmath.*;

public class Stochastic
{
    /** 
     * A list of points showing the path of the object after the impulse is applied.
     * This should be populated when this Stochastic object is constructed.
     */
    public ArrayList<ArrayList<Point2d>> paths = new ArrayList<ArrayList<Point2d>>();
    public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>(); 
    
    public Color3f displayColor = new Color3f(Color.RED); 
    public Color3f highlightColor = new Color3f(Color.GREEN);
    
    public int chosenIndex = -1; 
    
    public Stochastic(RigidBody body)
    {

    	for(int i = 0; i < Constants.NUM_PATHS; i++)
    	{
    		paths.add(new ArrayList<Point2d>()); 
    	}
    	for(int i = 0; i < Constants.NUM_PATHS; i++)
    	{
    		RigidBody tempBody = new RigidBody(body); 
    		RigidBodySystem.copyBody(tempBody,body); 
    		bodies.add(tempBody); 
    	}
    	
    }

    
    /** 
     * Displays the object to the screen, given a color.
     * By default, draws the path as line segments.
     */
    public void display(GL2 gl) {
    	gl.glColor3f(displayColor.x,displayColor.y,displayColor.z);

    	for(ArrayList<Point2d> path : paths)
    	{
    		gl.glBegin(GL2.GL_LINES);
    		for (int i=0; i<path.size()-1; i++) {
    			gl.glVertex2d(path.get(i).x, path.get(i).y);
    		}
    		gl.glEnd();
    	}
    	if(chosenIndex != -1)
    	{
    		//Change color of highlighted path
    		gl.glBegin(GL2.GL_LINES);
    		gl.glColor3f(highlightColor.x,highlightColor.y,highlightColor.z);
    		ArrayList<Point2d> highlightPath =  paths.get(chosenIndex);
    		for (int i = 0; i<highlightPath.size()-1; i++)
    		{
    			gl.glVertex2d(highlightPath.get(i).x,highlightPath.get(i).y); 
    		}
    		gl.glEnd();

    	}
    }

    public void displayChosenPath(GL2 gl)
    {
    	gl.glColor3f(highlightColor.x, highlightColor.y, highlightColor.z);
    	if(chosenIndex != -1)
    	{
    		ArrayList<Point2d> displayPath= paths.get(chosenIndex);

    		gl.glBegin(GL2.GL_LINES);
    		for (int i=0; i<displayPath.size()-1; i++) {
    			gl.glVertex2d(displayPath.get(i).x, displayPath.get(i).y);
    		}
    		gl.glEnd();
    	}
    }
    
    public void eliminatePaths(SelectionBox sb)
    {
    	Iterator<ArrayList<Point2d>> Piter = paths.iterator();
    	Iterator<RigidBody> RBiter = bodies.iterator();
    	boolean boxIntersected; 
    	while (Piter.hasNext()) {
    	    ArrayList<Point2d> path = Piter.next();
    	    RBiter.next();
    	    boxIntersected = false; 
    	    for(Point2d p : path)
    	    {
    	    	if(sb.pointWithinBox(p))
    	    	{
    	    		boxIntersected = true;
    	    		break;
    	    	}
    	    }
    	    if(!boxIntersected)
    	    {
    	    	Piter.remove();
    	    	RBiter.remove(); 
    	    }
    	}
    }

    /** Returns true if a given point intersects the path of the Stochastic object. Useful for selecting paths. */
    public int intersectsPath(Point2d p, double tolerance) {
    	for(ArrayList<Point2d> path : paths)
    	{
    		for (int i=0; i<path.size()-1; i++) {
    			if (Utils.distPointToSegment(p, path.get(i), path.get(i+1)) < tolerance) return paths.indexOf(path);
    		}
    	}
	return -1;
    }

    
}