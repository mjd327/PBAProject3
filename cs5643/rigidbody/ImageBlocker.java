package cs5643.rigidbody;

import java.io.*;
import java.nio.*;
import java.util.*;
import javax.vecmath.*;
import javax.media.opengl.*;
import com.jogamp.opengl.util.texture.spi.TGAImage;

/**
 * Converts an image into Block objects, and identifies connected
 * components to generate RigidBody objects.  
 * 
 * NOTE: YOU DON'T HAVE TO (or want to!) MODIFY THIS CODE.
 * 
 * @author Doug James, March 2007.
 */
public class ImageBlocker 
{
    /** Image in question. */
    TGAImage image;

    /** #Rows */
    int M;
    /** #Columns */
    int N;

    /** Block[i][j] with null for massless/background Blocks. */
    Block[][] B;

    /** Resulting sets of connected Block components. */
    ArrayList<HashSet<Block>> components = new ArrayList<HashSet<Block>>();

    /**
     *
     *
     * @param tgaFilename TGA image filename (RGB, uncompressed)
     * containing rigid image elements on a white background.
     */
    public ImageBlocker(String tgaFilename, GLProfile glp) throws IOException
    {
	image = loadImage(tgaFilename, glp);

	buildBlocks();

	findConnectedComponents();
    }

    /** Nonempty Blocks of image. */
    public ArrayList<Block> getBlocks() 
    {
	ArrayList<Block> all = new ArrayList<Block>();
	for(int i=0; i<M; i++) {
	    for(int j=0; j<N; j++) {
		if(B[i][j] != null)  all.add(B[i][j]);
	    }
	}
	return all;
    }

    /** Constructs new RigidBody objects for each connected component
     * of Blocks, and adds each to the RigidBodySystem. */
    public void addComponentRigidBodies(RigidBodySystem R) 
    {
    	RigidBody temp;
    	for(HashSet<Block> cc : components) 
    	{
    		temp = new RigidBody(cc);
    		
    		for(Block b: cc)
    		{
    			b.body = temp;
    		}
    		R.add(temp);
    		
    	}
    }

    private void findConnectedComponents()
    {
	//CCBuilder ccb = new CCBuilder();
	ScanlineCCBuilder ccb = new ScanlineCCBuilder();
    }

//     /** 
//      * Finds connected components. 
//      * @deprecated SLOW! ... use scanline version with expansion
//      * moves.
//      */
//     class CCBuilder
//     {
// 	HashSet<Block> remainingBlocks = new HashSet<Block>();

// 	CCBuilder() 
// 	{
// 	    remainingBlocks.addAll(getBlocks());

// 	    /// FIND CONNECTED COMPONENTS:
// 	    System.out.println("\nFINDING CONNECTED COMPONENTS...");
// 	    while(remainingBlocks.size() > 0) {/// FIND A CONNECTED COMPONENT:
// 		Block block = remainingBlocks.iterator().next();
// 		HashSet<Block> cc = findCC(block);
// 		remainingBlocks.removeAll(cc);
// 		components.add(cc);
// 		System.out.println("   |COMPONENT| = "+cc.size()); 
		
// 	    }
// 	    System.out.println("FOUND "+components.size()+" CONNECTED COMPONENTS");
// 	}

// 	HashSet<Block> findCC(Block root)
// 	{
// 	    HashSet<Block> cc       = new HashSet<Block>();
// 	    HashSet<Block> frontier = new HashSet<Block>();

// 	    frontier.add(root);
// 	    HashSet<Block> neighbors = new HashSet<Block>();
// 	    while(frontier.size() > 0) {
// 		Block block = frontier.iterator().next();//sloth

// 		// ADD BLOCK:
// 		cc.add(block);
// 		frontier.remove(block);

// 		// EXPAND NEIGHBORS:
// 		addBlockNeighbors(block, neighbors);
// 		neighbors.removeAll(cc);///guarantees neighbors are new (sloth)
// 		frontier.addAll(neighbors);
// 	    }

// 	    return cc;
// 	}

// 	void addBlockNeighbors(Block block, Collection<Block> addToMe) //, Set<Block> removeFromMe)
// 	{
// 	    int I = block.i();
// 	    int J = block.j();
// 	    for(int i=Math.max(0,I-1); i<=Math.min(I+1,M-1); i++) {
// 		for(int j=Math.max(0,J-1); j<=Math.min(J+1,N-1); j++) {
// 		    if(B[i][j] != null) {
// 			addToMe.add(B[i][j]);
// 			//removeFromMe.remove(B[i][j]);
// 		    }
// 		}
// 	    }
// 	    addToMe.remove(block);
// 	    //removeFromMe.remove(block);
// 	}
//     }

    /** 
     * Finds connected components by unifying row/column fragment
     * labels via iterative minimization of row/col fragment
     * labels using expansion moves.
     */
    class ScanlineCCBuilder
    {
	private ArrayList[] rowFrag; // of Frag
	private ArrayList[] colFrag; // of Frag
	private int[][]     label;

	ScanlineCCBuilder() 
	{
	    /// FIND CONNECTED COMPONENTS:
	    System.out.println("\nFINDING CONNECTED COMPONENTS...");

	    buildRowFragments();// --> rowFrag[i]
	    buildColFragments();// --> colFrag[j]

	    minimizeConnectedLabels();

	    extractComponents(); // --> components

	    System.out.println("FOUND "+components.size()+" RIGID BODIES");
	}

	/** Iterative minimization of connected labels using
	 * alternating row/column fragment minimization. */
	void minimizeConnectedLabels() 
	{
	    /// INITIALIZE LABELS: 
	    label = new int[M][N];
	    for(int i=0; i<M; i++) 
		for(int j=0; j<N; j++) 
		    label[i][j] = Integer.MAX_VALUE;

	    /// INIT TO ROW FRAG/COMPONENT LABELS:
	    int fragCount = 0;
	    for(int i=0; i<M; i++) {
		ArrayList<RowFrag> frags = (ArrayList<RowFrag>)rowFrag[i];
		for(RowFrag frag : frags) {
		    for(Block block : frag.getBlocks()) {
			label[block.i()][block.j()] = fragCount;
		    }
		    fragCount++;
		}
	    }

	    /// ITERATE ROW/COLUMN LABEL MINIMIZATION (a simple
	    /// "expansion move" ... a fun thing to do at Cornell):
	    int nChangesThisIteration = 1;
	    while(nChangesThisIteration > 0) {/// STOP IF EXPANSION
					      /// MOVE HAS NO CHANGES
		nChangesThisIteration = 0;

		/// ROW EXPANSION:
		for(ArrayList<RowFrag> frags : rowFrag) {
		    for(RowFrag frag : frags) {
			int     minLabel = frag.getMinLabel();
			boolean changed  = frag.setMinLabel(minLabel);
			if(changed) nChangesThisIteration++;
		    }
		}

		/// COL EXPANSION:
		for(ArrayList<ColFrag> frags : colFrag) {
		    for(ColFrag frag : frags) {
			int     minLabel = frag.getMinLabel();
			boolean changed  = frag.setMinLabel(minLabel);
			if(changed) nChangesThisIteration++;
		    }
		}

		System.out.println("ITERATE: #label changes = "+nChangesThisIteration);
	    }
	}

	/** Extracts "components" from label */
	void extractComponents()
	{
	    // MAP label-->blocks
	    HashMap<Integer,HashSet<Block>> map = new HashMap<Integer,HashSet<Block>>();
	    
	    for(int i=0; i<M; i++) {
		for(int j=0; j<N; j++) {
		    int lab = label[i][j];
		    if(lab < Integer.MAX_VALUE) {//valid label:
			HashSet<Block> comp = map.get(lab);
			if(comp==null) {
			    comp = new HashSet<Block>();
			    map.put(lab, comp);
			}
			comp.add(B[i][j]);
		    }
		}
	    }
	    
	    // EXTRACT COMPONENTS:
	    HashSet<HashSet<Block>> values = new HashSet<HashSet<Block>>();
	    values.addAll(map.values());
	    for(HashSet<Block> set : values) {
		components.add(set);
	    }
	}

	void buildColFragments()
	{
	    colFrag = new ArrayList[N];

 	    System.out.print("BUILDING COL FRAGMENTS ");
	    int count = 0;
	    for(int j=0; j<N; j++) {
		colFrag[j] = new ArrayList<ColFrag>();

 		ColFrag fragment = null;
		for(int i=0; i<M; i++) {
 		    if(B[i][j] == null) {
 			fragment = null;
 		    }
 		    else {
 			if(fragment==null)   {
			    fragment = new ColFrag(j);
			    colFrag[j].add(fragment);
			    count++;
			}

 			fragment.add(B[i][j]);
 		    }
 		}
 		if(j%(N/10)==0) System.out.print(".");
 	    }
 	    System.out.println(" DONE ("+count+" col fragments)");
	}

	void buildRowFragments()
	{
	    rowFrag = new ArrayList[M];

 	    System.out.print("BUILDING ROW FRAGMENTS ");
	    int count = 0;
 	    for(int i=0; i<M; i++) {
		rowFrag[i] = new ArrayList<RowFrag>();

 		RowFrag fragment = null;
 		for(int j=0; j<N; j++) {
 		    if(B[i][j] == null) {
 			fragment = null;
 		    }
 		    else {
 			if(fragment==null)   {
			    fragment = new RowFrag(i);
			    rowFrag[i].add(fragment);
			    count++;
			}

 			fragment.add(B[i][j]);
 		    }
 		}
		if(rowFrag[i].size() > 0) System.out.print(" "+rowFrag[i].size()+" ");
 		if(i%(M/10)==0) System.out.print(".");
 	    }
 	    System.out.println(" DONE ("+count+" row fragments)");
	}

	class RowFrag
	{
	    ArrayList<Block> blocks = new ArrayList<Block>();
	    int index;

	    ///index=i
	    RowFrag(int index) { this.index = index; }

	    public void add(Block block) { blocks.add(block); }

	    public ArrayList<Block> getBlocks() { return blocks; }

	    int  getMinLabel()
	    {
		int min = Integer.MAX_VALUE;
		for(Block block : blocks) { 
		    int j = block.j();
		    min = Math.min(min, label[index][j]);
		    if(index > 0) {//Check previous row too (faster propagation)
			min = Math.min(min, label[index-1][j]);
			/// NOTE: Effectively ignores B=null entries, since
			/// they have label=Integer.MAX_VALUE

			if(j>0)   min = Math.min(min, label[index-1][j-1]);
			if(j+1<N) min = Math.min(min, label[index-1][j+1]);
		    }

// 		    if(index+1 < M) {//Check previous row too (faster propagation)
// 			min = Math.min(min, label[index+1][j]);
// 			/// NOTE: Effectively ignores B=null entries, since
// 			/// they have label=Integer.MAX_VALUE

// 			if(j>0)   min = Math.min(min, label[index+1][j-1]);
// 			if(j+1<N) min = Math.min(min, label[index+1][j+1]);
// 		    }
		}
		return min;
	    }

	    /**
	     * @return True if some values changed.
	     */
	    boolean setMinLabel(int min) {
		boolean changed = false;
		for(Block block : blocks)  {
		    if(label[index][block.j()] != min)  changed = true;
		    label[index][block.j()] = min;
		}
		return changed;
	    }
	}

	/** Col impl of Frag */
	class ColFrag
	{
	    ArrayList<Block> blocks = new ArrayList<Block>();
	    int index;//col

	    ColFrag(int index) { this.index = index; }

	    public void add(Block block) { blocks.add(block); }

	    public ArrayList<Block> getBlocks() { return blocks; }

	    int  getMinLabel()
	    {
		int min = Integer.MAX_VALUE;
		for(Block block : blocks) { 
		    int i = block.i();
		    min = Math.min(min, label[i][index]);
		    if(index > 0) {//Check previous col too (faster propagation)
			min = Math.min(min, label[i][index-1]);
			/// NOTE: Effectively ignores B=null entries, since
			/// they have label=Integer.MAX_VALUE

			if(i > 0)  min = Math.min(min, label[i-1][index-1]);
			if(i+1<M)  min = Math.min(min, label[i+1][index-1]);
		    }

 		    if(index+1 < N) {
 			min = Math.min(min, label[i][index+1]);
 			if(i > 0)  min = Math.min(min, label[i-1][index+1]);
 			if(i+1<M)  min = Math.min(min, label[i+1][index+1]);
 		    }



		}
		return min;
	    }

	    boolean setMinLabel(int min) {
		boolean changed = false;
		for(Block block : blocks)  {
		    if(label[block.i()][index] != min) changed = true;
		    label[block.i()][index] = min;
		}
		return changed;
	    }
	}//ColFrag
    }

    /** Builds nonwhite blocks. */
    private void buildBlocks()
    {
	//int nx = ; 
	M = image.getHeight();
	N = image.getWidth();
	B = new Block[M][N];

	int maxMN = (int)Math.max(M,N);
	double  h = 0.5/(double)maxMN;//halfwidth 
	ByteBuffer bytes = image.getData();
	bytes.rewind();
	int nnz = 0;
	for(int i=0; i<M; i++) {
	    for(int j=0; j<N; j++) {
		double  x = h*(2*j + 1);
		double  y = h*(2*i + 1);
		Point2d p = new Point2d(x,y);
		Color3f c = nextColor(bytes);

		/// RECORD MASSIVE BLOCKS:
		Block block = new Block(i, j, c, p, h);
		if(block.getColorMass() > 0.01) {
		    B[i][j] = block;
		    nnz++;
		}
	    }
	}
	System.out.println("Built "+nnz+" blocks / "+(M*N)+" or "+((float)nnz/(float)(M*N)*100.f)+"%");
    }

    private static TGAImage loadImage(String tgaFilename, GLProfile glp) throws IOException
    {
	TGAImage image = TGAImage.read(glp, tgaFilename);
	System.out.println("image('"+tgaFilename+"'): height="+image.getHeight()+" width="+image.getWidth());
	System.out.println("Format: " + image.getGLFormat() +"; needed "+ GL2.GL_BGR);
	
	// FIXME: might want to accept GL2.GL_BGRA as well
	if(image.getGLFormat() != GL2.GL_BGR) 
	    throw new RuntimeException("Only BGR TGA formats accepted.");

	return image;
    }


    /** Parses next BGR triple. */
    private static Color3f nextColor(ByteBuffer bytes) 
    {
	float b = nextChannel(bytes);
	float g = nextChannel(bytes);
	float r = nextChannel(bytes);
	/// FIXME: a channel is image-dependent!
	//	float a = nextChannel(bytes);
	return new Color3f(r,g,b);
    }
    private static float nextChannel(ByteBuffer bytes) 
    {
	int   r = (int)bytes.get();
	r = (r + 256) % 256;
	float x = (float)r/255.f;
	if(x > 1.f) x = 1.f;
	if(x < 0.f) x = 0.f;
	//System.out.println("r = "+r+" --> x="+x);
	return x;
    }
}
