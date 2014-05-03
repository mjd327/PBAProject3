package cs5643.rigidbody;

import java.util.*;
import javax.vecmath.*;
import javax.media.opengl.*;

/**
 * Representation of a circular bounding disk.
 *
 * @author Doug James, March 2007.
 */
public class Disk
{
    static int DISK_DISPLAY_LIST = -1;

    /** Center of disk. */
    Point2d c;

    /** Radius of disk. */
    double  r;

    /** Builds a Disk that encloses the specified blocks in their
     * frame of reference.
     * 
     * (HINT: You might get better performance from an AABB,
     * especially for text)  
     */
    Disk(Collection<Block> blocks)
    {
	////////////////////////////////
	/// ;P  TEMP: LOOSE-FITTING Disk: 
	////////////////////////////////
	/// 1. Center at average position 
	Vector2d mean = new Vector2d();
	for(Block block : blocks) {
	    mean.add(block.p());
	}
	mean.scale(1./(double)blocks.size());
	c = new Point2d(mean);

	/// 2. FIND SMALLEST BOUNDING RADIUS FOR THIS CENTER:
	r = 0;
	for(Block block : blocks) {
	    r = Math.max(r, c.distance(block.p()) + 2*block.h());//conservative block radius, 2h
	}
    }

    /** Builds Disk with specified center and radius values. */
    Disk(Point2d center, double radius) 
    { 
	c = new Point2d(center);
	r = radius;
    } 

    /** Fragile reference to center of Disk. */
    Point2d c() { return c; }

    /** Radius of Disk. */
    public double r() { return r; }

    /** 
     * True if point intersects this bounding volume.
     * @param p Point in frame of reference of Disk. 
     */
    public boolean intersects(Point2d p)
    {
	return (p.distanceSquared(c) <= r*r);
    }

    /** Draws circular disk using a display list. */
    public void display(GL2 gl)
    {
	displayDisk(gl, c, r);
    }

    synchronized static void displayDisk(GL2 gl, Tuple2d c, double r) 
    {
	if(DISK_DISPLAY_LIST < 0) {// MAKE DISPLAY LIST:
	    int displayListIndex = gl.glGenLists(1);
	    gl.glNewList(displayListIndex, GL2.GL_COMPILE);
	    drawDisk(gl); /// Unit disk at origin
	    gl.glEndList();
	    System.out.println("MADE DISK LIST "+displayListIndex+" : "+gl.glIsList(displayListIndex));
	    DISK_DISPLAY_LIST = displayListIndex;
	}

	/// COLOR: DEFAULT WHITE
	//gl.glColor3f(1,1,1);

	/// DRAW ORIGIN-CIRCLE TRANSLATED TO c, and scaled by r
	gl.glPushMatrix();
	gl.glTranslated(c.x, c.y, 0);
	gl.glScaled(r,r,r);
	gl.glCallList(DISK_DISPLAY_LIST);
	gl.glPopMatrix();
    }
    
    /** 
     * Draws a canonical unit disk at the origin.
     */
    private static void drawDisk(GL2 gl)
    {
	double  radius = 1;//unit
	Point2d p      = new Point2d();//origin

	double vectorY1 = p.y;
	double vectorX1 = p.x;
 
	gl.glBegin(GL2.GL_TRIANGLES);
	int N = 20;
	for(int i=0; i<=N; i++)
	    {
		double angle   = ((double)i) * 2. * Math.PI / (double)N;
		double vectorX = p.x + radius*Math.sin(angle);
		double vectorY = p.y + radius*Math.cos(angle);
		gl.glVertex2d(p.x,p.y);
		gl.glVertex2d(vectorX1,vectorY1);
		gl.glVertex2d(vectorX,vectorY);
		vectorY1 = vectorY;
		vectorX1 = vectorX;	
	    }
	gl.glEnd();
    }




}
