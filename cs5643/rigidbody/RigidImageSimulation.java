package cs5643.rigidbody;

import java.io.*;
import java.util.*;

import java.awt.*;
import java.awt.event.*;

import javax.swing.*;
import javax.vecmath.*;

import javax.media.opengl.*;
import javax.media.opengl.awt.GLCanvas;
import com.jogamp.opengl.util.*;



/**
 * 
 * CS5643: Assignment #3 "Rigid Body Contact" main
 * class. RigidImageSimulation simulates rigid bodies derived from an
 * image's connected nonwhite components.
 * 
 * @author Doug James, March 2007, 2009
 */
public class RigidImageSimulation implements GLEventListener, MouseListener, MouseMotionListener
{
    /** Reference to current FrameExporter, or null if no frames being
     * dumped. */
    FrameExporter frameExporter;

    /** Number of time steps per large step size. */
    static int N_STEPS_PER_FRAME = 15;

    /** Size of symplectic Euler time step (in seconds). You may want
     * to change this, but watch out for interpenetrations at high
     * linear/angular speeds! */
    public static double DT = 0.0001;

    /** Main window frame. */
    JFrame frame = null;

    private int width, height;

    /** The single RigidBodySystem reference. */
    RigidBodySystem RBS;

    /** Toggle to advance simulation. */
    boolean generating      = false;
    boolean choosing        = false; 
    boolean animating       = false; 
    
    /** Draws wireframe if true, and pixel blocks if false. */
    boolean drawWireframe = false;

    /** Draws object bounds (Disk) if true. */
    boolean drawBounds    = false;

    /** If true, instead of one step per frame, simulator takes
     * N_STEPS_PER_FRAME. */
    boolean largeStep     = true;

    /**Always store the mouse clicks, it may come in handy for selection.*/
    Point2d mouseStart = new Point2d(); 
    
    //Booleans to control user interaction. 
    boolean firstStep = true; 
    boolean spaceEnabled = true; 
    boolean displayFinalPaths = true; 
    /** 
     * Main constructor. Call start() to begin simulation. 
     * 
     * @param imageFilename Image to simulate.
     * @param processCollisions Enables collisions if true.
     */
    RigidImageSimulation(String imageFilename, boolean processCollisions) 
	throws IOException
    {
	RBS = new RigidBodySystem();
	RBS.setProcessCollisions(processCollisions);

	ImageBlocker blocker = new ImageBlocker(imageFilename, GLProfile.getDefault());
	blocker.addComponentRigidBodies(RBS);

	/// CONSTRAIN RIGID BODIES THAT ARE BLUE:
	for(RigidBody b : RBS.getRigidBodies()) {
	    if(b.hasAllBlueBlocks())
		b.setPin(true);
	}

	//System.out.println("imageFilename = "+imageFilename);
    }

    /**
     * Builds and shows window, and starts simulator.
     */
    public void start()
    {
	if(frame != null) return;

	frame = new JFrame("CS5643 Rigid Body Contact!");
	GLProfile glp = GLProfile.getDefault();
	GLCapabilities caps = new GLCapabilities(glp);
	GLCanvas canvas = new GLCanvas(caps);
	canvas.addGLEventListener(this);
	frame.add(canvas);

 	canvas.addMouseListener(this);
 	canvas.addMouseMotionListener(this);

 	canvas.addKeyListener(new KeyAdapter() {
		public void keyTyped(KeyEvent e) { } // NOP
 		public void keyPressed(KeyEvent e) {
 		    dispatchKey(e.getKeyChar(), e);
 		}
		public void keyReleased(KeyEvent e) { } // NOP
 	    });


	final Animator animator = new Animator(canvas);
	frame.addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent e) {
		    // Run this on another thread than the AWT event queue to
		    // make sure the call to Animator.stop() completes before
		    // exiting
		    new Thread(new Runnable() {
			    public void run() {
				animator.stop();
				System.exit(0);
			    }
			}).start();
		}
	    });

	frame.pack();
	frame.setSize(720,742);/// DEFAULT 720x720 FRAME SIZE
	frame.setLocation(200, 0);
	frame.setVisible(true);
	animator.start();
    }



    private OrthoMap orthoMap;

    /** Maps mouse event into computational cell using OrthoMap. */
    public Point2d getPoint2d(MouseEvent e) {
	return orthoMap.getPoint2d(e);
    }

    /** GLEventListener implementation: Initializes JOGL renderer. */
    public void init(GLAutoDrawable drawable) 
    {
	// DEBUG PIPELINE (can use to provide GL error feedback... disable for speed)
	//drawable.setGL(new DebugGL(drawable.getGL()));

	GL2 gl = drawable.getGL().getGL2();
	System.err.println("INIT GL IS: " + gl.getClass().getName());

	gl.setSwapInterval(1);

	gl.glLineWidth(1);
	gl.glPointSize(1f);

	//gl.glDisable(gl.GL_DEPTH_TEST);

	gl.glEnable(GL2.GL_BLEND);
	gl.glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
	//gl.glBlendFunc(GL2.GL_SRC_ALPHA_SATURATE, GL2.GL_ONE_MINUS_SRC_ALPHA);
	//gl.glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE);
	gl.glEnable(GL2.GL_POINT_SMOOTH);
	gl.glHint  (GL2.GL_POINT_SMOOTH_HINT,  GL2.GL_NICEST);
	gl.glEnable(GL2.GL_LINE_SMOOTH);
	gl.glHint  (GL2.GL_LINE_SMOOTH_HINT,   GL2.GL_NICEST);
	gl.glEnable(GL2.GL_POLYGON_SMOOTH); 
	gl.glHint  (GL2.GL_POLYGON_SMOOTH_HINT,GL2.GL_NICEST);

    }

    /** GLEventListener implementation */
    public void displayChanged(GLAutoDrawable drawable, boolean modeChanged, boolean deviceChanged) {}

    /** GLEventListener implementation */
    public void dispose(GLAutoDrawable drawable) {}

    /** GLEventListener implementation */
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) 
    {
	System.out.println("width="+width+", height="+height);
	height = Math.max(height, 1); // avoid height=0;
	
	this.width  = width;
	this.height = height;

 	GL2 gl = drawable.getGL().getGL2();
	gl.glViewport(0,0,width,height);	

	/// SETUP ORTHOGRAPHIC PROJECTION AND MAPPING INTO UNIT CELL:
	gl.glMatrixMode(GL2.GL_PROJECTION);	
	gl.glLoadIdentity();			
	orthoMap = new OrthoMap(width, height);//Hide grungy details in OrthoMap
	orthoMap.apply_glOrtho(gl);

	/// GET READY TO DRAW:
	gl.glMatrixMode(GL2.GL_MODELVIEW);
	gl.glLoadIdentity();
    }


    /** 
     * Main event loop: OpenGL display + simulation
     * advance. GLEventListener implementation.
     */
    public void display(GLAutoDrawable drawable) 
    {
	GL2 gl = drawable.getGL().getGL2();
	gl.glClearColor(1,1,1,0);
	gl.glClear(GL2.GL_COLOR_BUFFER_BIT); //  | GL.GL_DEPTH_BUFFER_BIT);

	{/// DRAW COMPUTATIONAL CELL BOUNDARY:
	    gl.glBegin(GL2.GL_LINE_STRIP);
	    if(generating)
		gl.glColor4f(0,0,0,1);
	    else 
		gl.glColor4f(1,0,0,1);
	    gl.glVertex2d(0,0);	gl.glVertex2d(1,0);	gl.glVertex2d(1,1);	gl.glVertex2d(0,1);	gl.glVertex2d(0,0);
	    gl.glEnd();
	}
            
	if (drawWireframe)  gl.glPolygonMode(GL2.GL_FRONT, GL2.GL_LINE);
	else                gl.glPolygonMode(GL2.GL_FRONT, GL2.GL_FILL);

	simulateAndDisplayScene(gl);/// <<<-- MAIN CALL


	if(frameExporter != null)  frameExporter.writeFrame(gl);
    }

    /** Simulate then display particle system and any builder
     * adornments. */
    void simulateAndDisplayScene(GL2 gl)
    {
    	if(generating) {/// TAKE DT step sizes... take N_STEPS_PER_FRAME if largeStep==true
    		displayFinalPaths= true; 
    		if(firstStep)
    		{
    			if(largeStep)
    			{
    				for(int k = 0; k < N_STEPS_PER_FRAME; k++)
    				{
    					firstStep = !generateInitialPath(DT); 
    				}
    			}
    			else
    			{
    				firstStep = !generateInitialPath(DT);
    			}

    		}
    		else {//JUST ONE DT STEP
    			if(spaceEnabled)
    			{
    				spaceEnabled = false; 
    				generatePaths(gl,DT);
    				spaceEnabled = true; 
    				generating = false;
    				choosing = true; 
    			}
    		}
    	}
    	
    	if(animating)
    	{
    		displayFinalPaths = false; 
			if(largeStep)
			{
				for(int k = 0; k < N_STEPS_PER_FRAME; k++)
				{
					animating = RBS.updateAnimation(); 
					if(!animating)
					{
						break; 
					}
				}
			}
			else
			{
				RBS.updateAnimation(); 
			}
    	}
    	// Draw particles, springs, etc.
    	RBS.display(gl,displayFinalPaths);

    	if(drawBounds) {
    		for(RigidBody body : RBS.getRigidBodies()) {
    			body.displayBound(gl);

    		}
    	}
    }


    
    
	//Calculate a new set of paths, and then display them. 
	void generatePaths(GL2 gl, double dt)
	{		
		RigidBody b = RBS.getUnpinnedBody(); 
		Stochastic s = new Stochastic(b); 
		RBS.S.add(s); 
		RBS.generatePaths(s,dt); 
	}
	
	//Generates a initial path when the program starts. 
	boolean generateInitialPath(double dt)
	{
		if(RBS.S.size() == 0)
		{
			RigidBody b = RBS.getUnpinnedBody();
			//Computes boundary blocks, because of strange code logic in RigidBody class
			b.getBoundaryBlocks(); 
			Stochastic s = new Stochastic(b); 
			RBS.S.add(s); 
		}
		return RBS.initialSimulation(dt); 
	}
	
	
    void advanceTime(double dt) 
    {
	RBS.advanceTime(DT);
    }

    private SpringForcePoint2Body mouseForce = null;

    // Methods required for the implementation of MouseListener
    public void mouseEntered (MouseEvent e) { }
    public void mouseExited  (MouseEvent e) { }
    public void mousePressed (MouseEvent e) 
    { 
    	//Always store the click no matter what.
    	mouseStart.set(getPoint2d(e));
    	RBS.removeForce(mouseForce);//in case stale


    }
    public void mouseReleased(MouseEvent e) { 
    	if(mouseForce != null) 	RBS.removeForce(mouseForce);
    	mouseForce = null; 
    	if(choosing && RBS.sb != null)
    	{
    		RBS.eliminatePaths(); 
    		//RBS.sb = null; 
    	}
    }
    public void mouseClicked (MouseEvent e) { 
    	if(mouseForce != null) 	RBS.removeForce(mouseForce);
    	mouseForce = null; 
    	if(choosing)
    	{
    		/// FIND A/CLOSEST BODY:
    		Point2d   p    = getPoint2d(e);
    		//Get the most recent stochastic and finds if the point intersects any paths 
    		Stochastic recentS = RBS.S.get(RBS.S.size()-1); 
    		int pathIndex = recentS.intersectsPath(p, Constants.MOUSE_TOLERANCE);
    		
    		if(pathIndex != -1)  {
    			if(recentS.chosenIndex != -1)
        		{
    				//You've chosen a path before, and need to remove its body
        			RigidBody prevb = recentS.bodies.get(recentS.chosenIndex);
        			RBS.remove(prevb); 
        		}
    			if(pathIndex == recentS.chosenIndex)
    			{
    				//You've selected current path, now deselect instead. 
    				recentS.chosenIndex = -1;
    			}
    			else
    			{
    			RigidBody b = recentS.bodies.get(pathIndex);  
    			RBS.add(b); 
      			recentS.chosenIndex = pathIndex; 
    			}
    		}
    	}
    }

    // Methods required for the implementation of MouseMotionListener
    public void mouseDragged (MouseEvent e) { 
    	if(mouseForce != null) {
    		Point2d p = getPoint2d(e);
    		mouseForce.updatePoint(p);
    		//System.out.print(" U ");
    	}
    	if(choosing)
    	{
    		Point2d p = getPoint2d(e);
    		if(RBS.sb == null) RBS.sb = new SelectionBox(mouseStart,p);
    		else RBS.sb.adjustSelectionBox(p); 
    		
    		
    	}
    }
    public void mouseMoved   (MouseEvent e) { }

    /**
     * Handles keyboard events, e.g., spacebar toggles
     * simulation/pausing, and escape resets the current Task.
     */
    public void dispatchKey(char key, KeyEvent e)
    {
	//System.out.println("CHAR="+key+", keyCode="+e.getKeyCode()+", e="+e);
	if(key == ' ') {//SPACEBAR --> TOGGLE SIMULATE
		if(choosing)
		{
			if(!generating)
			{
				RBS.sb = null; 
				Stochastic prior = RBS.S.get(RBS.S.size()-1);

				if(prior.chosenIndex == -1)
				{
					//Not yet chosen, regenerate
					RBS.S.remove(RBS.S.size()-1); 
					prior = RBS.S.get(RBS.S.size()-1);
					RBS.add(prior.bodies.get(prior.chosenIndex));
				}
			}
		}
		generating = true; 
		
	}
	else if (key == 'a')
	{
		if(choosing)
		{
			choosing = false; 
			animating = true; 
			
		}
	}
	else if (key == 'r') {//RESET
	    System.out.println("RESET!");
	    generating = false;
	    choosing = false; 
	    frameExporter = null;
	    RBS.reset();
	}
	else if (key == 'w') {//RESET
	    //System.out.println("WIREFRAME");
	    drawWireframe = !drawWireframe;
	}
	else if (key == 'b') {//RESET
	    drawBounds = !drawBounds;
	    //System.out.println("drawBounds = "+drawBounds);
	}
	else if (key == 'l') {//LARGE STEP TOGGLE
	    largeStep = !largeStep;
	    if(largeStep) 
		System.out.println("MULTI-STEP");
	    else 
		System.out.println("SINGLE-STEP");
	}
	else if (key == 'j') {//RANDOM JIGGLES
	    System.out.println("JIGGLING");
	    RBS.jiggle();
	}
	else if (e.toString().contains("Escape")) {//sloth
	    //System.out.println("ESCAPE");
	}
	else if (key == 'e') {//toggle exporter
	    frameExporter = ((frameExporter==null) ? (new FrameExporter()) : null);
	    System.out.println("'e' : frameExporter = "+frameExporter);
	}
    }


    /**
     * Code to dump frames---very useful for slow/large runs. 
     */
    private static int exportId = -1;
    private class FrameExporter
    {
	private int nFrames  = 0;

	FrameExporter()  { 
	    exportId += 1;
	}

	void writeFrame(GL2 gl)
        {
            long   timeNS   = -System.nanoTime();
            String number   = Utils.getPaddedNumber(nFrames, 5, "0");
            String filename = "frames/export"+exportId+"-"+number+".png"; /// Bug: DIRECTORY MUST EXIST!

            try{
                java.io.File   file     = new java.io.File(filename);
                if(file.exists()) System.out.println("WARNING: OVERWRITING PREVIOUS FILE: "+filename);

		GLReadBufferUtil rbu = new GLReadBufferUtil(false, false);
		rbu.readPixels(gl, false);
		rbu.write(file);

                System.out.println((timeNS/1000000)+"ms:  Wrote frame: "+filename);

            }catch(Exception e) {
		e.printStackTrace();
                System.out.println("OOPS: "+e);
            }

            nFrames += 1;
	}

    }


    /**
     * ### Runs the RigidImageSimulation. ###
     */
    public static void main(String[] args) 
    {
	try{
	    String  dynamicFilename   = "images/test.tga";
	    if(args.length >= 1) {
		dynamicFilename = args[0];
	    }
	    System.out.println("Image = args[0] = "+dynamicFilename);

	    boolean processCollisions = true;
	    RigidImageSimulation  sim = new RigidImageSimulation(dynamicFilename, processCollisions);
	    sim.start();

	}catch(Exception e) {
	    e.printStackTrace();
	    System.out.println("OOPS: "+e);
	}
    }
}
