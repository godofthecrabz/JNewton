package crab.newton;

import crab.newton.callbacks.NewtonCollisionTreeRayCastCallback;
import crab.newton.callbacks.NewtonTreeCollisionFaceCallback;
import crab.newton.internal.*;
import java.lang.foreign.*;

public final class NewtonTreeCollision implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonTreeCollision(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param shapeID
	 * @return
	 */
	public static NewtonTreeCollision create(NewtonWorld world, int shapeID) {
		return new NewtonTreeCollision(Newton_h.NewtonCreateTreeCollision(world.address, shapeID));
	}
	
	public static NewtonTreeCollision createFromMesh(NewtonWorld world, NewtonMesh mesh, int shapeID) {
		return new NewtonTreeCollision(Newton_h.NewtonCreateTreeCollisionFromMesh(world.address, mesh.address, shapeID));
	}
	
	public void setUserRayCastCallback(NewtonCollisionTreeRayCastCallback rayHitCallback, MemorySession session) {
		MemorySegment rayHitCallbackFunc = NewtonCollisionTreeRayCastCallback.allocate(rayHitCallback, session);
		Newton_h.NewtonTreeCollisionSetUserRayCastCallback(address, rayHitCallbackFunc);
	}
	
	public void beginBuild() {
		Newton_h.NewtonTreeCollisionBeginBuild(address);
	}
	
	public void addFace(int vertexCount, float[] vertexList, int strideInBytes, int faceAttribute) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment vertexSegment = session.allocateArray(Newton_h.C_FLOAT, vertexList);
			Newton_h.NewtonTreeCollisionAddFace(address, vertexCount, vertexSegment, strideInBytes, faceAttribute);
		}
	}
	
	public void endBuild(int optimize) {
		Newton_h.NewtonTreeCollisionEndBuild(address, optimize);
	}
	
	public int getFaceAttribute(int[] faceIndexArray, int indexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indexSegment = session.allocateArray(Newton_h.C_INT, faceIndexArray);
			return Newton_h.NewtonTreeCollisionGetFaceAttribute(address, indexSegment, indexCount);
		}
	}
	
	public void setFaceAttribute(int[] faceIndexArray, int indexCount, int attribute) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indexSegment = session.allocateArray(Newton_h.C_INT, faceIndexArray);
			Newton_h.NewtonTreeCollisionSetFaceAttribute(address, indexSegment, indexCount, attribute);
		}
	}
	
	public void forEachFace(NewtonTreeCollisionFaceCallback forEachFaceCallback, Addressable context, MemorySession session) {
		MemorySegment forEachFunc = NewtonTreeCollisionFaceCallback.allocate(forEachFaceCallback, session);
		Newton_h.NewtonTreeCollisionForEachFace(address, forEachFunc, context);
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
