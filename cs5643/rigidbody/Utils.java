package cs5643.rigidbody;
import javax.vecmath.*;

/** 
 * Catch-all utilities (feel free to add on). 
 * 
 * @author Doug James, January 2007
 */
public class Utils
{

    /**  Returns (ax*by-ay*bx).  */
    static double crossZ(Tuple2d a, Tuple2d b)  {  return (a.x*b.y - a.y*b.x);   }

    /**
     * sum += scale*v
     */
    public static void acc(Tuple2d sum, double scale, Tuple2d v)
    {
	sum.x += scale * v.x;
	sum.y += scale * v.y;
    }


    /**
     * 
     * @param pad Pre-padding char.
     */
    public static String getPaddedNumber(int number, int length, String pad)  {
	return getPaddedString(""+number, length, pad, true);
    }

    /**
     * @param prePad Pre-pads if true, else post pads.
     */
    public static String getPaddedString(String s, int length, String pad, boolean prePad) 
    {
	if(pad.length() != 1) throw new IllegalArgumentException("pad must be a single character.");
	String result = s;
	result.trim();
	if(result.length() > length) 
	    throw new RuntimeException
		("input string "+s+" is already more than length="+length);

	int nPad = (length - result.length());
	for(int k=0; k<nPad; k++) {
	    //System.out.println("nPad="+nPad+", result="+result+", result.length()="+result.length());
	    if(prePad) 
		result = pad + result;
	    else
		result = result + pad;
	}

	return result;
    }

    /**
     * Calculates the distance between a point and a line segment.
     * @param p The point.
     * @param p0 The first end of the segment.
     * @param p1 The second end of the segment.
     */
    public static double distPointToSegment(Point2d p, Point2d p0, Point2d p1)
    {
	Vector2d v = new Vector2d();
	Vector2d w = new Vector2d();
	v.sub(p1, p0);
	w.sub(p,  p0);

	double c1 = w.dot(v);
	if (c1 <= 0) return p.distance(p0);

	double c2 = v.lengthSquared();
	if (c2 <= c1) return p.distance(p1);

	Point2d pb = new Point2d();
	pb.scaleAdd(c1 / c2, v, p0);
	return p.distance(pb);
    }
}
