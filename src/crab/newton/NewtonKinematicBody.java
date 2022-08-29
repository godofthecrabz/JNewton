package crab.newton;

import crab.newton.internal.Newton_h;
import jdk.incubator.foreign.*;

public final class NewtonKinematicBody implements NewtonBody {
	
	private final MemoryAddress address;
	
	protected NewtonKinematicBody(MemoryAddress address) {
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
	public static NewtonKinematicBody create(NewtonWorld world, NewtonCollision collision, float[] matrix, SegmentAllocator allocator) {
		MemorySegment matrixSegment = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
		return new NewtonKinematicBody(Newton_h.NewtonCreateKinematicBody(world.address, collision.address(), matrixSegment));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
