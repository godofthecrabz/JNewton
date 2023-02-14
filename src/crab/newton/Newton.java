package crab.newton;

import crab.newton.internal.Platform;
import crab.newton.internal.RuntimeHelper;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.RandomAccessFile;
import java.lang.foreign.MemorySession;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Newton {
	
	protected static final float[] VEC3F = new float[3];
	protected static final double[] VEC3D = new double[3];
	protected static final float[] VEC4F = new float[4];
	protected static final float[] AABBF = new float[6];
	protected static final float[] MAT4F = new float[16];
	protected static final float[] MAT4F_VEC3F = new float[19];
	
	private Newton() {}

	/**
	 * Attempts to load Newton library from jar.
	 * Method fails if library jar i
	 * @throws IOException
	 */
	public static void loadNewton(MemorySession session) throws IOException {
		RuntimeHelper.loadLibrary(session);
	}
	
	public static void loadNewton(String filepath, MemorySession session) {
		RuntimeHelper.loadLibrary(filepath, session);
	}

	public static boolean unloadNewton() {
		return RuntimeHelper.unloadLibrary();
	}
}
