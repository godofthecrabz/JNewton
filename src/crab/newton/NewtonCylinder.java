package crab.newton;

import crab.newton.internal.Newton_h;
import jdk.incubator.foreign.*;

public final class NewtonCylinder implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonCylinder(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param radio0
	 * @param radio1
	 * @param height
	 * @param shapeID
	 * @param offsetMatrix
	 * @param allocator
	 * @return
	 */
	public static NewtonCylinder create(NewtonWorld world, float radio0,  float radio1,  float height,  int shapeID,  float[] offsetMatrix, SegmentAllocator allocator) {
		if (offsetMatrix == null) {
			return new NewtonCylinder(Newton_h.NewtonCreateCylinder(world.address, radio0, radio1, height, shapeID, MemoryAddress.NULL));
		}
		MemorySegment matrix = allocator.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
		return new NewtonCylinder(Newton_h.NewtonCreateCylinder(world.address, radio0, radio1, height, shapeID, matrix));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
