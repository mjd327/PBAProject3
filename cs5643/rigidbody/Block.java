package cs5643.rigidbody;

import java.awt.Color;

import javax.vecmath.*;
import javax.media.opengl.*;

/** 
 * Basic square block/pixel primitive for representing image-based
 * rigid objects and resolving contacts. Each Block is wrapped by a
 * disk and used for contact generation.
 * 
 * @author Doug James, March 2007. 
 */
public class Block 
{
	/** draw block a different color */
	boolean highlighted;
	
	/** Image row index. */
	int     i;

	/** Image column index. */
	int     j;

	/** Pixel color. */
	Color3f c;

	/** Body-frame position --- needed for contact processing. */
	Point2d p;

	/** Halfwidth of block (Note: the block radius is sqrt(2)*h). */
	double  h;

	/** Body this block belongs too */
	RigidBody body;
	/**
	 * Constructs a Block.
	 */
	Block(int i, int j, Color3f color, Point2d center, double halfwidth)
	{
		this.i  = i;
		this.j  = j;
		this.c  = new Color3f(color);
		this.p  = new Point2d(center);
		this.h  = halfwidth;
	}

	/** Image row. */
	int i() { return i; } 

	/** Image column. */
	int j() { return j; } 

	/** Center position of Block (in body coordinates). */
	public Point2d p() { return p; }

	/** Halfwidth of block. Note that the block radius is sqrt(2)*h.  */
	public double  h() { return h; }

	/** Color-based mass on [0,1] with white having zero mass, and
	 * darker colors approaching one (feel free to modify). */
	public double getColorMass() 
	{
		double m = 1 - ((double)(c.x + c.y + c.z))/3.0; // on [0,1]
		if(m < 0) m = 0;
		if(m > 1) m = 1;
		return m;
	}

	public void setBody(RigidBody body) { this.body = body; }

	/** Draws Block geometry (using current color/lighting). */
	public void display(GL2 gl) 
	{
		if(highlighted)
		{
			System.out.println("draw highlighted block");
			gl.glBegin(GL2.GL_QUADS);
			gl.glColor3f(Color.RED.getRed(), Color.RED.getGreen(), Color.RED.getBlue());
			gl.glVertex2d(p.x-h, p.y-h);
			gl.glVertex2d(p.x+h, p.y-h);
			gl.glVertex2d(p.x+h, p.y+h);
			gl.glVertex2d(p.x-h, p.y+h);
			gl.glEnd();
		}
		else
		{
			double r = 0.5*h;
			gl.glBegin(GL2.GL_QUADS);
			gl.glColor3f(c.x, c.y, c.z);
			gl.glVertex2d(p.x-h, p.y-h);
			gl.glVertex2d(p.x+h, p.y-h);
			gl.glVertex2d(p.x+h, p.y+h);
			gl.glVertex2d(p.x-h, p.y+h);
			gl.glEnd();
		}
	}
}
