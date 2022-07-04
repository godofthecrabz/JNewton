package crab.newton;

import crab.newton.generated.RuntimeHelper;

public class Newton {
	
	protected static final float[] VEC3F = new float[3];
	protected static final double[] VEC3D = new double[3];
	protected static final float[] VEC4F = new float[4];
	protected static final float[] AABBF = new float[6];
	protected static final float[] MAT4F = new float[16];
	protected static final float[] MAT4F_VEC3F = new float[19];
	
	private Newton() {}
	
	public static void loadNewtonAbsolute(String filepath) {
		RuntimeHelper.loadLibrary(filepath);
	}
}
