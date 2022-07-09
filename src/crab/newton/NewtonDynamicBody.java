package crab.newton;

import crab.newton.generated.Newton_h;

import jdk.incubator.foreign.*;

public final class NewtonDynamicBody implements NewtonBody {
	
	private final MemoryAddress address;
	
	protected NewtonDynamicBody(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param collision
	 * @param matrix
	 * @param allocator
	 * @return
	 */
	public static NewtonDynamicBody create(NewtonWorld world, NewtonCollision collision, float[] matrix, SegmentAllocator allocator) {
		MemorySegment matrixSegment = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
		return new NewtonDynamicBody(Newton_h.NewtonCreateDynamicBody(world.address, collision.address(), matrixSegment));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
