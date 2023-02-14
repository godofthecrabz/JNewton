package crab.newton;

import crab.newton.callbacks.NewtonCollisionTreeRayCastCallback;
import crab.newton.callbacks.NewtonHeightFieldRayCastCallback;
import crab.newton.callbacks.NewtonTreeCollisionFaceCallback;
import crab.newton.internal.Newton_h;
import java.lang.foreign.*;

public final class NewtonCollision {

	public static final int SPHERE = 0,
			CAPSULE = 1,
			CYLINDER = 2,
			CHAMFERCYLINDER = 3,
			BOX = 4,
			CONE = 5,
			CONVEXHULL = 6,
			NULL = 7,
			COMPOUND = 8,
			TREE = 9,
			HEIGHTFIELD = 10,
			CLOTH_PATCH = 11,
			DEFORMABLE_SOLID = 12,
			USERMESH = 13,
			SCENE = 14,
			FRACTURED_COMPOUND = 15;
	protected final MemoryAddress address;
	public final int collisionType;

	protected NewtonCollision(MemoryAddress address) {
		this(address, Newton_h.NewtonCollisionGetType(address));
	}

	protected NewtonCollision(MemoryAddress address, int collisionType) {
		this.address = address;
		this.collisionType = collisionType;
	}
	
	public int getMode() {
		return Newton_h.NewtonCollisionGetMode(address);
	}

	public void setMode(int mode) {
		Newton_h.NewtonCollisionSetMode(address, mode);
	}

	public float calculateVolume() {
		return Newton_h.NewtonConvexCollisionCalculateVolume(address);
	}

	public float[] calculateInertiaMatrix() {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment inertiaOrigin = session.allocateArray(Newton_h.C_FLOAT, Newton.AABBF);
			Newton_h.NewtonConvexCollisionCalculateInertialMatrix(address,
					inertiaOrigin.asSlice(0L, Newton_h.C_FLOAT.byteSize() * 3), 
					inertiaOrigin.asSlice(Newton_h.C_FLOAT.byteSize() * 3));
			return inertiaOrigin.toArray(Newton_h.C_FLOAT);
		}
	}

	public boolean isConvex() {
		return Newton_h.NewtonCollisionIsConvexShape(address) == 1 ? true : false;
	}

	public boolean isStatic() {
		return Newton_h.NewtonCollisionIsStaticShape(address) == 1 ? true : false;
	}

	public void setUserData(Addressable data) {
		Newton_h.NewtonCollisionSetUserData(address, data);
	}

	public MemoryAddress getUserData() {
		return Newton_h.NewtonCollisionGetUserData(address);
	}

	public void setUserID(long id) {
		Newton_h.NewtonCollisionSetUserID(address, id);
	}

	public long getUserID() {
		return Newton_h.NewtonCollisionGetUserID(address);
	}

	public MemoryAddress getSubCollisionHandle() {
		return Newton_h.NewtonCollisionGetSubCollisionHandle(address);
	}

	public NewtonCollision getParentInstance() {
		MemoryAddress ptr = Newton_h.NewtonCollisionGetParentInstance(address);
		return ptr.equals(MemoryAddress.NULL) ? null : NewtonCollision.wrap(ptr);
	}

	public void setMatrix(float[] matrix) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrixSegment = session.allocateArray(Newton_h.C_FLOAT, matrix);
			Newton_h.NewtonCollisionSetMatrix(address, matrixSegment);
		}
	}

	public float[] getMatrix() {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrixSegment = session.allocateArray(Newton_h.C_FLOAT, new float[16]);
			Newton_h.NewtonCollisionGetMatrix(address, matrixSegment);
			return matrixSegment.toArray(Newton_h.C_FLOAT);
		}
	}

	public void setScale(float x, float y, float z) {
		Newton_h.NewtonCollisionSetScale(address, x, y, z);
	}

	public float[] getScale() {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment xyzSeg = session.allocateArray(Newton_h.C_FLOAT, new float[3]);
			Newton_h.NewtonCollisionGetScale(address,
					xyzSeg.asSlice(0L, Newton_h.C_FLOAT.byteSize()), 
					xyzSeg.asSlice(4L, Newton_h.C_FLOAT.byteSize()), 
					xyzSeg.asSlice(8L, Newton_h.C_FLOAT.byteSize()));
			return xyzSeg.toArray(Newton_h.C_FLOAT);
		}
	}

	public void destroy() {
		Newton_h.NewtonDestroyCollision(address);
	}

	public float getSkinThickness() {
		return Newton_h.NewtonCollisionGetSkinThickness(address);
	}

	public void setSkinThickness(float thickness) {
		Newton_h.NewtonCollisionSetSkinThickness(address, thickness);
	}

	//Compound & Scene Collision methods

	public void beginAddRemove() {
		switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionBeginAddRemove(address);
			case SCENE -> Newton_h.NewtonSceneCollisionBeginAddRemove(address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public MemoryAddress addSubCollision(NewtonCollision collision) {
		return switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionAddSubCollision(address, collision.address);
			case SCENE -> Newton_h.NewtonSceneCollisionAddSubCollision(address, collision.address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public void removeSubCollision(MemoryAddress collisionNode) {
		switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionRemoveSubCollision(address, collisionNode);
			case SCENE -> Newton_h.NewtonSceneCollisionRemoveSubCollision(address, collisionNode);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public void removeSubCollisionByIndex(int index) {
		switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionRemoveSubCollisionByIndex(address, index);
			case SCENE -> Newton_h.NewtonSceneCollisionRemoveSubCollisionByIndex(address, index);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public void setSubCollisionMatrix(MemoryAddress collisionNode, float[] matrix) {
		//switch (collisionType) {
		//	case COMPOUND -> Newton_h.NewtonCompoundCollisionRemoveSubCollisionByIndex(address, index);
		//	case SCENE -> Newton_h.NewtonSceneCollisionRemoveSubCollisionByIndex(address, index);
		//	default -> throw new RuntimeException("Collision Type incompatible with this method");
		//}
	}

	public void endAddRemove() {
		switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionEndAddRemove(address);
			case SCENE -> Newton_h.NewtonSceneCollisionEndAddRemove(address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public MemoryAddress getFirstNode() {
		return switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionGetFirstNode(address);
			case SCENE -> Newton_h.NewtonSceneCollisionGetFirstNode(address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public MemoryAddress getNextNode(MemoryAddress nextNode) {
		return switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionGetNextNode(address, nextNode);
			case SCENE -> Newton_h.NewtonSceneCollisionGetNextNode(address, nextNode.address());
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public MemoryAddress getNodeByIndex(int index) {
		return switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionGetNodeByIndex(address, index);
			case SCENE -> Newton_h.NewtonSceneCollisionGetNodeByIndex(address, index);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public int getNodeIndex(MemoryAddress collisionNode) {
		return switch (collisionType) {
			case COMPOUND -> Newton_h.NewtonCompoundCollisionGetNodeIndex(address, collisionNode);
			case SCENE -> Newton_h.NewtonSceneCollisionGetNodeIndex(address, collisionNode);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public NewtonCollision getCollisionFromNode(MemoryAddress collisionNode) {
		return switch (collisionType) {
			case COMPOUND -> NewtonCollision.wrap(Newton_h.NewtonCompoundCollisionGetCollisionFromNode(address, collisionNode));
			case SCENE -> NewtonCollision.wrap(Newton_h.NewtonSceneCollisionGetCollisionFromNode(address, collisionNode));
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	//Convex Hull collision methods

	public int getFaceIndices(int face, int[] faceIndices) {
		return 0;
	}

	//HeightField collision methods

	public void setUserRaycastCallback(NewtonHeightFieldRayCastCallback rayHitCallback, MemorySession session) {
		if (collisionType != HEIGHTFIELD) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		MemorySegment rayHitCallbackFunc = NewtonHeightFieldRayCastCallback.allocate(rayHitCallback, session);
		Newton_h.NewtonHeightFieldSetUserRayCastCallback(address, rayHitCallbackFunc);
	}

	//Tree Collision methods

	public void setUserRayCastCallback(NewtonCollisionTreeRayCastCallback rayHitCallback, MemorySession session) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		MemorySegment rayHitCallbackFunc = NewtonCollisionTreeRayCastCallback.allocate(rayHitCallback, session);
		Newton_h.NewtonTreeCollisionSetUserRayCastCallback(address, rayHitCallbackFunc);
	}

	public void beginBuild() {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton_h.NewtonTreeCollisionBeginBuild(address);
	}

	public void addFace(int vertexCount, float[] vertexList, int strideInBytes, int faceAttribute) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment vertexSegment = session.allocateArray(Newton_h.C_FLOAT, vertexList);
			Newton_h.NewtonTreeCollisionAddFace(address, vertexCount, vertexSegment, strideInBytes, faceAttribute);
		}
	}

	public void endBuild(int optimize) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton_h.NewtonTreeCollisionEndBuild(address, optimize);
	}

	public int getFaceAttribute(int[] faceIndexArray, int indexCount) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indexSegment = session.allocateArray(Newton_h.C_INT, faceIndexArray);
			return Newton_h.NewtonTreeCollisionGetFaceAttribute(address, indexSegment, indexCount);
		}
	}

	public void setFaceAttribute(int[] faceIndexArray, int indexCount, int attribute) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indexSegment = session.allocateArray(Newton_h.C_INT, faceIndexArray);
			Newton_h.NewtonTreeCollisionSetFaceAttribute(address, indexSegment, indexCount, attribute);
		}
	}

	public void forEachFace(NewtonTreeCollisionFaceCallback forEachFaceCallback, Addressable context, MemorySession session) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		MemorySegment forEachFunc = NewtonTreeCollisionFaceCallback.allocate(forEachFaceCallback, session);
		Newton_h.NewtonTreeCollisionForEachFace(address, forEachFunc, context);
	}
	
	public static NewtonCollision wrap(MemoryAddress address) {
		int collisionType = Newton_h.NewtonCollisionGetType(address);
		return new NewtonCollision(address, collisionType);
	}
}