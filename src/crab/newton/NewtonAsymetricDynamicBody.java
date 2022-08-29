package crab.newton;

import crab.newton.internal.Newton_h;
import jdk.incubator.foreign.*;

public final class NewtonAsymetricDynamicBody implements NewtonBody {
	
	private final MemoryAddress address;

	protected NewtonAsymetricDynamicBody(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * Creates a new {@code NewtonAsymetricDynamicBody}
	 * @param world - {@code NewtonWorld} instance
	 * @param collision - {@code NewtonCollision} shape
	 * @param matrix - 16 float array that contains a 4x4 matrix in row-major order
	 * @param allocator - {@code SegmentAllocator} used to allocate matrix
	 * @return new {@code NewtonAsymetricDynamicBody}
	 */
	public static NewtonAsymetricDynamicBody create(NewtonWorld world, NewtonCollision collision, float[] matrix, SegmentAllocator allocator) {
		MemorySegment matrixSeg = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
		return new NewtonAsymetricDynamicBody(Newton_h.NewtonCreateAsymetricDynamicBody(world.address, collision.address(), matrixSeg));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
