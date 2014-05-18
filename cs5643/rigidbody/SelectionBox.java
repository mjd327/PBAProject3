package cs5643.rigidbody;

import java.awt.Color;
import java.util.ArrayList;

import javax.media.opengl.GL2;
import javax.vecmath.Color3f;
import javax.vecmath.Point2d;

public class SelectionBox {
	/**The fop4 corners of the rectangle.*/
	Point2d p1 = new Point2d();
	Point2d p2 = new Point2d();
	Point2d p3 = new Point2d();
	Point2d p4 = new Point2d();
	
	String movablePoint = ""; 
	/**The fop4 corners of the rectangle.*/
	Color3f outlineColor = new Color3f(Color.BLACK);
	Color3f fillColor = new Color3f(Color.GREEN); 

	double xMin;
	double xMax;
	double yMin;
	double yMax; 
	
	public SelectionBox(Point2d p1, Point2d p2)
	{
		//Calculate the other points of the rectangle.
		this.p1.set(p1);
		this.p2.set(p2);
		this.p3.set(p1.x,p2.y);
		this.p4.set(p2.x,p1.y);
		
		xMin = Math.min(Math.min(p1.x,p2.x),p3.x);
		yMin = Math.min(Math.min(p1.y,p2.y),p3.y);
		xMax = Math.max(Math.max(p1.x,p2.x),p3.x);
		yMax = Math.max(Math.max(p1.y,p2.y),p3.y);

	}
	/**Used to change the selection box as the mouse drags.*/
	public void adjustSelectionBox(Point2d p)
	{
		this.p2.set(p);
		this.p3.set(p1.x,p2.y);
		this.p4.set(p2.x,p1.y);

		xMin = Math.min(Math.min(p1.x,p2.x),p3.x);
		yMin = Math.min(Math.min(p1.y,p2.y),p3.y);
		xMax = Math.max(Math.max(p1.x,p2.x),p3.x);
		yMax = Math.max(Math.max(p1.y,p2.y),p3.y);
	}
	/**Display both the outline and center of the selection box*/ 
	public void displaySelectionBox(GL2 gl)
	{
		gl.glColor3f(outlineColor.x,outlineColor.y,outlineColor.z);
		//Draw the lines
		gl.glBegin(GL2.GL_LINES);
		gl.glVertex2d(p1.x,p1.y);
		gl.glVertex2d(p3.x,p3.y);
		gl.glVertex2d(p3.x,p3.y);
		gl.glVertex2d(p2.x,p2.y);
		gl.glVertex2d(p2.x,p2.y);
		gl.glVertex2d(p4.x,p4.y);
		gl.glVertex2d(p4.x,p4.y);
		gl.glVertex2d(p1.x,p1.y);
		gl.glEnd();

		//Draw the middle of the box
		gl.glColor4f(fillColor.x,fillColor.y,fillColor.z,.3f);
		gl.glBegin(GL2.GL_QUADS);
		gl.glVertex2d(p1.x,p1.y);
		gl.glVertex2d(p3.x,p3.y);
		gl.glVertex2d(p2.x,p2.y);
		gl.glVertex2d(p4.x,p4.y);
		gl.glEnd();
	}
	
	/**Checks if a point intersects this selection box*/
	public boolean pointWithinBox(Point2d p)
	{
		
		return(p.x  > xMin && p.x < xMax && p.y < yMax && p.y > yMin);
	}
}


