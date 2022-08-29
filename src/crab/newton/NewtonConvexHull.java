package crab.newton;

import crab.newton.internal.Newton_h;
import jdk.incubator.foreign.*;

public final class NewtonConvexHull implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonConvexHull(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param count
	 * @param vertexCloud
	 * @param strideInBytes
	 * @param tolerance
	 * @param shapeID
	 * @param offsetMatrix
	 * @param allocator
	 * @return
	 */
	public static NewtonConvexHull create(NewtonWorld world, int count,  float[] vertexCloud,  int strideInBytes,  float tolerance,  int shapeID,  float[] offsetMatrix, SegmentAllocator allocator) {
		MemorySegment vertCloud = allocator.allocateArray(Newton_h.C_FLOAT, vertexCloud);
		if (offsetMatrix == null) {
			return new NewtonConvexHull(Newton_h.NewtonCreateConvexHull(world.address, count, vertCloud, strideInBytes, tolerance, shapeID, MemoryAddress.NULL));
		}
		MemorySegment matrix = allocator.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
		return new NewtonConvexHull(Newton_h.NewtonCreateConvexHull(world.address, count, vertCloud, strideInBytes, tolerance, shapeID, matrix));
	}
	
	public int getFaceIndices(int face, int[] faceIndices) {
		return 0;
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
