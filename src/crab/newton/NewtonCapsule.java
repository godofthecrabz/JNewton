package crab.newton;

import crab.newton.generated.*;

import jdk.incubator.foreign.*;

public final class NewtonCapsule implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonCapsule(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param radius0
	 * @param radius1
	 * @param height
	 * @param shapeID
	 * @param offsetMatrix
	 * @param allocator
	 * @return
	 */
	public static NewtonCapsule create(NewtonWorld world, float radius0, float radius1, float height, int shapeID, float[] offsetMatrix, SegmentAllocator allocator) {
		if (offsetMatrix == null) {
			return new NewtonCapsule(Newton_h.NewtonCreateCapsule(world.address, radius0, radius1, height, shapeID, MemoryAddress.NULL));
		}
		MemorySegment matrix = allocator.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
		return new NewtonCapsule(Newton_h.NewtonCreateCapsule(world.address, radius0, radius1, height, shapeID, matrix));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
