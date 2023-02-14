package crab.newton;

import crab.newton.callbacks.NewtonAllocMemory;
import crab.newton.callbacks.NewtonBodyIterator;
import crab.newton.callbacks.NewtonCollisionCopyConstructionCallback;
import crab.newton.callbacks.NewtonCollisionDestructorCallback;
import crab.newton.callbacks.NewtonContactsProcess;
import crab.newton.callbacks.NewtonCreateContactCallback;
import crab.newton.callbacks.NewtonDeserializeCallback;
import crab.newton.callbacks.NewtonDestroyContactCallback;
import crab.newton.callbacks.NewtonFreeMemory;
import crab.newton.callbacks.NewtonIslandUpdate;
import crab.newton.callbacks.NewtonJobTask;
import crab.newton.callbacks.NewtonJointIterator;
import crab.newton.callbacks.NewtonOnAABBOverlap;
import crab.newton.callbacks.NewtonOnBodyDeserializationCallback;
import crab.newton.callbacks.NewtonOnBodySerializationCallback;
import crab.newton.callbacks.NewtonOnCompoundSubCollisionAABBOverlap;
import crab.newton.callbacks.NewtonOnContactGeneration;
import crab.newton.callbacks.NewtonOnJointDeserializationCallback;
import crab.newton.callbacks.NewtonOnJointSerializationCallback;
import crab.newton.callbacks.NewtonPostUpdateCallback;
import crab.newton.callbacks.NewtonSerializeCallback;
import crab.newton.callbacks.NewtonWorldDestroyListenerCallback;
import crab.newton.callbacks.NewtonWorldDestructorCallback;
import crab.newton.callbacks.NewtonWorldListenerBodyDestroyCallback;
import crab.newton.callbacks.NewtonWorldListenerDebugCallback;
import crab.newton.callbacks.NewtonWorldRayFilterCallback;
import crab.newton.callbacks.NewtonWorldRayPrefilterCallback;
import crab.newton.callbacks.NewtonWorldUpdateListenerCallback;
import crab.newton.internal.*;
import java.lang.foreign.*;
import java.nio.charset.StandardCharsets;

/**
 * 
 * @author Christopher
 *
 */
public class NewtonWorld {
	
	private static final Object[] EMPTY = new Object[] {};
	protected final MemoryAddress address;
	
	/**
	 * Internal NewtonWorld Constructor
	 * @param address - address of the NewtonWorld object
	 */
	private NewtonWorld(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * Creates a NewtonWorld instance
	 * @return NewtonWorld
	 */
	public static NewtonWorld create() {
		return new NewtonWorld(Newton_h.NewtonCreate(EMPTY));
	}

	/**
	 * Creates an instance of a box collision
	 * @param dx
	 * @param dy
	 * @param dz
	 * @param shapeID
	 * @param offsetMatrix
	 * @return
	 */
	public NewtonCollision createBox(float dx, float dy, float dz, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton_h.NewtonCreateBox(address, dx, dy, dz, shapeID, matrix));
		}
	}

	public NewtonCollision createBox(float dx, float dy, float dz, int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateBox(address, dx, dy, dz, shapeID, MemoryAddress.NULL));
	}

	/**
	 *
	 * @param radius0
	 * @param radius1
	 * @param height
	 * @param shapeID
	 * @param offsetMatrix
	 * @return
	 */
	public NewtonCollision createCapsule(float radius0, float radius1, float height, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton_h.NewtonCreateCapsule(address, radius0, radius1, height, shapeID, matrix));
		}
	}

	public NewtonCollision createCapsule(float radius0, float radius1, float height, int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateCapsule(address, radius0, radius1, height, shapeID, MemoryAddress.NULL));
	}

	/**
	 *
	 * @param radius
	 * @param height
	 * @param shapeID
	 * @param offsetMatrix
	 * @return
	 */
	public NewtonCollision createChamferCylinder(float radius,  float height,  int shapeID,  float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton_h.NewtonCreateChamferCylinder(address, radius, height, shapeID, matrix));
		}
	}

	public NewtonCollision createChamferCylinder(float radius,  float height,  int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateChamferCylinder(address, radius, height, shapeID, MemoryAddress.NULL));
	}

	public NewtonCollision createCompoundCollision(int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateCompoundCollision(address, shapeID));
	}

	public NewtonCollision createCompoundCollision(NewtonMesh mesh, float hullTolerance, int shapeID, int subShapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateCompoundCollisionFromMesh(address, mesh.address, hullTolerance, shapeID, subShapeID));
	}

	public NewtonCollision createCone(float radius,  float height,  int shapeID,  float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton_h.NewtonCreateCone(address, radius, height, shapeID, matrix));
		}
	}

	public NewtonCollision createCone(float radius,  float height,  int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateCone(address, radius, height, shapeID, MemoryAddress.NULL));
	}

	public NewtonCollision createConvexHull(int count,  float[] vertexCloud,  int strideInBytes,  float tolerance,  int shapeID,  float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			MemorySegment vertCloud = session.allocateArray(Newton_h.C_FLOAT, vertexCloud);
			return new NewtonCollision(Newton_h.NewtonCreateConvexHull(address, count, vertCloud, strideInBytes, tolerance, shapeID, matrix));
		}
	}

	public NewtonCollision createConvexHull(int count,  float[] vertexCloud,  int strideInBytes,  float tolerance,  int shapeID) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment vertCloud = session.allocateArray(Newton_h.C_FLOAT, vertexCloud);
			return new NewtonCollision(Newton_h.NewtonCreateConvexHull(address, count, vertCloud, strideInBytes, tolerance, shapeID, MemoryAddress.NULL));
		}
	}

	public NewtonCollision createCylinder(float radio0,  float radio1,  float height,  int shapeID,  float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton_h.NewtonCreateCylinder(address, radio0, radio1, height, shapeID, matrix));
		}
	}

	public NewtonCollision createCylinder(float radio0,  float radio1,  float height,  int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateCylinder(address, radio0, radio1, height, shapeID, MemoryAddress.NULL));
	}

	public NewtonCollision createHeightField(int width,  int height,  int gridsDiagonals,  float[] elevationMap,  char[] attributeMap,  float verticalScale,  float horizontalScale_x,  float horizontalScale_z,  int shapeID) {
		int elevationDataType = 0;
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment elevationMapSegment = session.allocateArray(Newton_h.C_FLOAT, elevationMap);
			MemorySegment attributeSeg = session.allocateArray(Newton_h.C_CHAR, new String(attributeMap).getBytes(StandardCharsets.UTF_8));
			return new NewtonCollision(Newton_h.NewtonCreateHeightFieldCollision(address, width, height, gridsDiagonals, elevationDataType, elevationMapSegment, attributeSeg, verticalScale, horizontalScale_x, horizontalScale_z, shapeID));
		}
	}

	public NewtonCollision createHeightField(int width,  int height,  int gridsDiagonals,  short[] elevationMap,  char[] attributeMap,  float verticalScale,  float horizontalScale_x,  float horizontalScale_z,  int shapeID) {
		int elevationDataType = 1;
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment elevationMapSegment = session.allocateArray(Newton_h.C_SHORT, elevationMap);
			MemorySegment attributeSeg = session.allocateArray(Newton_h.C_CHAR, new String(attributeMap).getBytes(StandardCharsets.UTF_8));
			return new NewtonCollision(Newton_h.NewtonCreateHeightFieldCollision(address, width, height, gridsDiagonals, elevationDataType, elevationMapSegment, attributeSeg, verticalScale, horizontalScale_x, horizontalScale_z, shapeID));
		}
	}

	public NewtonCollision createNullCollision() {
		return new NewtonCollision(Newton_h.NewtonCreateNull(address));
	}

	public NewtonCollision createSceneCollision(int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateSceneCollision(address, shapeID));
	}

	public NewtonCollision createSphere(float radius, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton_h.NewtonCreateSphere(address, radius, shapeID, matrix));
		}
	}

	public NewtonCollision createSphere(float radius, int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateSphere(address, radius, shapeID, MemoryAddress.NULL));
	}

	public NewtonCollision createTreeCollision(int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateTreeCollision(address, shapeID));
	}

	public NewtonCollision createTreeCollisionFromMesh(NewtonMesh mesh, int shapeID) {
		return new NewtonCollision(Newton_h.NewtonCreateTreeCollisionFromMesh(address, mesh.address, shapeID));
	}

	public NewtonBody createAsymetricDynamicBody(NewtonCollision collision, float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrixSeg = session.allocateArray(Newton_h.C_FLOAT, matrix);
			return  new NewtonBody(Newton_h.NewtonCreateAsymetricDynamicBody(address, collision.address, matrixSeg));
		}
	}

	public NewtonBody createDynamicBody(NewtonCollision collision, float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrixSeg = session.allocateArray(Newton_h.C_FLOAT, matrix);
			return  new NewtonBody(Newton_h.NewtonCreateDynamicBody(address, collision.address, matrixSeg));
		}
	}

	public NewtonBody createKinematicBody(NewtonCollision collision, float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrixSeg = session.allocateArray(Newton_h.C_FLOAT, matrix);
			return  new NewtonBody(Newton_h.NewtonCreateKinematicBody(address, collision.address, matrixSeg));
		}
	}
	
	/**
	 * Gets the version of Newton
	 * @return Newton API Version
	 */
	public static int getWorldVersion() {
		return Newton_h.NewtonWorldGetVersion(EMPTY);
	}
	
	/**
	 * Gets the size of floats in Newton library
	 * @return size of floats in bytes
	 */
	public static int getFloatSizes() {
		return Newton_h.NewtonWorldFloatSize(EMPTY);
	}
	
	/**
	 * Gets the amount of memory used by Newton
	 * @return amount of memory used in bytes
	 */
	public static int getMemoryUsed() {
		return Newton_h.NewtonGetMemoryUsed(EMPTY);
	}
	
	/**
	 * Set the memory system for Newton library
	 * @param alloc - allocation function
	 * @param free - deallocation function
	 * @param session - ResourceScope for allocating upcall stubs
	 */
	public static void setMemorySystem(NewtonAllocMemory alloc, NewtonFreeMemory free, MemorySession session) {
		MemorySegment allocFunc = NewtonAllocMemory.allocate(alloc, session);
		MemorySegment freeFunc = NewtonFreeMemory.allocate(free, session);
		Newton_h.NewtonSetMemorySystem(allocFunc, freeFunc);
	}
	
	/**
	 * Allocates memory with Newton allocator
	 * @param sizeInBytes - size in bytes of memory to be allocated
	 * @return MemoryAddress to the allocated memory
	 */
	public static MemoryAddress newtonAlloc(int sizeInBytes) {
		return Newton_h.NewtonAlloc(sizeInBytes);
	}
	
	/**
	 * Allocates memory with Newton allocator
	 * @param sizeInBytes - size in bytes of memory to be allocated
	 * @param session - ResourceScope for the returned MemorySegment
	 * @return MemorySegment representing the allocated memory
	 */
	public static MemorySegment newtonAlloc(int sizeInBytes, MemorySession session) {
		return MemorySegment.ofAddress(Newton_h.NewtonAlloc(sizeInBytes), sizeInBytes, session);
	}
	
	/**
	 * Allocates memory with Newton allocator and given MemoryLayout
	 * @param layout - MemoryLayout to be allocated
	 * @return MemoryAddress to the allocated memory
	 */
	public static MemoryAddress newtonAlloc(MemoryLayout layout) {
		return Newton_h.NewtonAlloc((int) layout.byteSize());
	}
	
	/**
	 * Allocates memory with Newton allocator and given MemoryLayout
	 * @param layout - MemoryLayout to be allocated
	 * @param session - ResourceScope for the returned MemorySegment
	 * @return MemorySegment representing the allocated memory
	 */
	public static MemorySegment newtonAlloc(MemoryLayout layout, MemorySession session) {
		return MemorySegment.ofAddress(Newton_h.NewtonAlloc((int) layout.byteSize()), layout.byteSize(), session);
	}
	
	/**
	 * Frees memory allocated by Newton
	 * @param ptr - pointer to the memory create by Newton
	 */
	public static void newtonFree(Addressable ptr) {
		Newton_h.NewtonFree(ptr);
	}
	
	/**
	 * Destroys the current Newton object.
	 */
	public void destroy() {
		Newton_h.NewtonDestroy(address);
	}
	
	public NewtonPostUpdateCallback getPostUpdateCallback(MemorySession session) {
		return NewtonPostUpdateCallback.ofAddress(Newton_h.NewtonGetPostUpdateCallback(address), session);
	}
	
	public void setPostUpdateCallback(NewtonPostUpdateCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonPostUpdateCallback.allocate(callback, session);
		Newton_h.NewtonSetPostUpdateCallback(address, callbackFunc);
	}
	
	public void loadPlugins(String pluginPath) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment path = session.allocateUtf8String(pluginPath);
			Newton_h.NewtonLoadPlugins(address, path);
		}
	}
	
	public void unloadPlugins() {
		Newton_h.NewtonUnloadPlugins(address);
	}
	
	public MemoryAddress getCurrentPlugin() {
		return Newton_h.NewtonCurrentPlugin(address);
	}
	
	public MemoryAddress getFirstPlugin() {
		return Newton_h.NewtonGetFirstPlugin(address);
	}
	
	public MemoryAddress getNextPlugin(MemoryAddress curPlugin) {
		return Newton_h.NewtonGetNextPlugin(address, curPlugin);
	}
	
	public MemoryAddress getPreferedPlugin() {
		return Newton_h.NewtonGetPreferedPlugin(address);
	}
	
	public float getContactMergeTolerance() {
		return Newton_h.NewtonGetContactMergeTolerance(address);
	}
	
	public void setContactMergeTolerance(float tolerance) {
		Newton_h.NewtonSetContactMergeTolerance(address, tolerance);
	}
	
	public void invalidateCache() {
		Newton_h.NewtonInvalidateCache(address);
	}
	
	public void setSolverIterations(int model) {
		Newton_h.NewtonSetSolverIterations(address, model);
	}
	
	public int getSolverIterations() {
		return Newton_h.NewtonGetSolverIterations(address);
	}
	
	public void setParallelSolverOnLargeIsland(int mode) {
		Newton_h.NewtonSetParallelSolverOnLargeIsland(address, mode);
	}
	
	public int getParallelSolverOnLargeIsland() {
		return Newton_h.NewtonGetParallelSolverOnLargeIsland(address);
	}
	
	public int getBroadphaseAlgorithm() {
		return Newton_h.NewtonGetBroadphaseAlgorithm(address);
	}
	
	public void selectBroadphaseAlgorithm(int algorithmType) {
		Newton_h.NewtonSelectBroadphaseAlgorithm(address, algorithmType);
	}
	
	public void resetBroadphase() {
		Newton_h.NewtonResetBroadphase(address);
	}
	
	public void update(float timestep) {
		Newton_h.NewtonUpdate(address, timestep);
	}
	
	public void updateAsync(float timestep) {
		Newton_h.NewtonUpdateAsync(address, timestep);
	}
	
	public void waitForUpdateToFinish() {
		Newton_h.NewtonWaitForUpdateToFinish(address);
	}
	
	public int getNumberOfSubsteps() {
		return Newton_h.NewtonGetNumberOfSubsteps(address);
	}
	
	public void setNumberOfSubsteps(int substeps) {
		Newton_h.NewtonSetNumberOfSubsteps(address, substeps);
	}
	
	public float getLastUpdateTime() {
		return Newton_h.NewtonGetLastUpdateTime(address);
	}
	
	public void serializeToFile(String filename, NewtonOnBodySerializationCallback bodyCallback, Addressable bodyUserData,
								MemorySession session) {
		MemorySegment filePath = session.allocateUtf8String(filename);
		MemorySegment callbackFunc = NewtonOnBodySerializationCallback.allocate(bodyCallback, session);
		Newton_h.NewtonSerializeToFile(address, filePath, callbackFunc, bodyUserData);
	}
	
	public void deserializeFromFile(String filename, NewtonOnBodyDeserializationCallback bodyCallback, Addressable bodyUserData,
									MemorySession session) {
		MemorySegment filePath = session.allocateUtf8String(filename);
		MemorySegment callbackFunc = NewtonOnBodyDeserializationCallback.allocate(bodyCallback, session);
		Newton_h.NewtonDeserializeFromFile(address, filePath, callbackFunc, bodyUserData);
	}
	
	public void serializeScene(NewtonOnBodySerializationCallback bodyCallback, Addressable bodyUserData, NewtonSerializeCallback serializeCallback, Addressable serializeHandle,
							   MemorySession session) {
		MemorySegment bodyFunc = NewtonOnBodySerializationCallback.allocate(bodyCallback, session);
		MemorySegment serializeFunc = NewtonSerializeCallback.allocate(serializeCallback, session);
		Newton_h.NewtonSerializeScene(address, bodyFunc, bodyUserData, serializeFunc, serializeHandle);
	}
	
	public void deserializeScene(NewtonOnBodyDeserializationCallback bodyCallback, Addressable bodyUserData, NewtonDeserializeCallback serializeCallback, Addressable serializeHandle,
								 MemorySession session) {
		MemorySegment bodyFunc = NewtonOnBodyDeserializationCallback.allocate(bodyCallback, session);
		MemorySegment deserializeFunc = NewtonDeserializeCallback.allocate(serializeCallback, session);
		Newton_h.NewtonDeserializeScene(address, bodyFunc, bodyUserData, deserializeFunc, serializeHandle);
	}
	
	public NewtonBody findSerializedBody(int bodySerializedID) {
		MemoryAddress body = Newton_h.NewtonFindSerializedBody(address, bodySerializedID);
		int bodyType = Newton_h.NewtonBodyGetType(body);
		return new NewtonBody(body, bodyType);
	}
	
	public void setJointSerializationCallbacks(NewtonOnJointSerializationCallback serializeJoint, NewtonOnJointDeserializationCallback deserializeJoint,
											   MemorySession session) {
		MemorySegment serializeFunc = NewtonOnJointSerializationCallback.allocate(serializeJoint, session);
		MemorySegment deserializeFunc = NewtonOnJointDeserializationCallback.allocate(deserializeJoint, session);
		Newton_h.NewtonSetJointSerializationCallbacks(address, serializeFunc, deserializeFunc);
	}
	
	public JointSerializationCallbacks getJointSerializationCallbacks(MemorySession session) {
		MemoryAddress serializePtr = MemoryAddress.NULL;
		MemoryAddress deserializePtr = MemoryAddress.NULL;
		Newton_h.NewtonGetJointSerializationCallbacks(address, serializePtr, deserializePtr);
		NewtonOnJointSerializationCallback serializeCallback = NewtonOnJointSerializationCallback.ofAddress(serializePtr, session);
		NewtonOnJointDeserializationCallback deserializeCallback = NewtonOnJointDeserializationCallback.ofAddress(deserializePtr, session);
		return new JointSerializationCallbacks(serializeCallback, deserializeCallback);
	}
	
	public void lockCriticalSection(int threadIndex) {
		Newton_h.NewtonWorldCriticalSectionLock(address, threadIndex);
	}
	
	public void unlockCriticalSection() {
		Newton_h.NewtonWorldCriticalSectionUnlock(address);
	}
	
	public void setThreadCount(int threads) {
		Newton_h.NewtonSetThreadsCount(address, threads);
	}
	
	public int getThreadCount() {
		return Newton_h.NewtonGetThreadsCount(address);
	}
	
	public int getMaxThreadCount() {
		return Newton_h.NewtonGetMaxThreadsCount(address);
	}
	
	public void dispatchThreadJob(NewtonJobTask task, Addressable userData, String functionName, MemorySession session) {
		MemorySegment function_name = session.allocateUtf8String(functionName);
		MemorySegment taskFunc = NewtonJobTask.allocate(task, session);
		Newton_h.NewtonDispachThreadJob(address, taskFunc, userData, function_name);
	}
	
	public void syncThreadJobs() {
		Newton_h.NewtonSyncThreadJobs(address);
	}
	
	public void setIslandUpdateEvent(NewtonIslandUpdate islandUpdate, MemorySession session) {
		MemorySegment islandUpdateFunc = NewtonIslandUpdate.allocate(islandUpdate, session);
		Newton_h.NewtonSetIslandUpdateEvent(address, islandUpdateFunc);
	}
	
	public void forEachJoint(NewtonJointIterator callback, Addressable userData, MemorySession session) {
		MemorySegment callbackFunc = NewtonJointIterator.allocate(callback, session);
		Newton_h.NewtonWorldForEachJointDo(address, callbackFunc, userData);
	}
	
	public void forEachBodyInAABB(float[] p0, float[] p1, NewtonBodyIterator callback, Addressable userData, MemorySession session) {
		MemorySegment p0Segment = session.allocateArray(Newton_h.C_FLOAT, p0);
		MemorySegment p1Segment = session.allocateArray(Newton_h.C_FLOAT, p1);
		MemorySegment callbackFunc = NewtonBodyIterator.allocate(callback, session);
		Newton_h.NewtonWorldForEachBodyInAABBDo(address, p0Segment, p1Segment, callbackFunc, userData);
	}
	
	public void setUserData(Addressable userData) {
		Newton_h.NewtonWorldSetUserData(address, userData);
	}
	
	public MemoryAddress getUserData() {
		return Newton_h.NewtonWorldGetUserData(address);
	}
	
	public MemoryAddress addListener(String nameId, Addressable listenerUserData, MemorySession session) {
		MemorySegment nameIdSegment = session.allocateUtf8String(nameId);
		return Newton_h.NewtonWorldAddListener(address, nameIdSegment, listenerUserData);
	}
	
	public MemoryAddress getListener(String nameId, MemorySession session) {
		MemorySegment nameIdSegment = session.allocateUtf8String(nameId);
		return Newton_h.NewtonWorldGetListener(address, nameIdSegment);
	}
	
	public void listenerSetDebugCallback(Addressable listener, NewtonWorldListenerDebugCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonWorldListenerDebugCallback.allocate(callback, session);
		Newton_h.NewtonWorldListenerSetDebugCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetPostStepCallback(Addressable listener, NewtonWorldUpdateListenerCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, session);
		Newton_h.NewtonWorldListenerSetPostStepCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetPreUpdateCallback(Addressable listener, NewtonWorldUpdateListenerCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, session);
		Newton_h.NewtonWorldListenerSetPreUpdateCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetPostUpdateCallback(Addressable listener, NewtonWorldUpdateListenerCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, session);
		Newton_h.NewtonWorldListenerSetPostUpdateCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetDestructorCallback(Addressable listener, NewtonWorldDestroyListenerCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonWorldDestroyListenerCallback.allocate(callback, session);
		Newton_h.NewtonWorldListenerSetDestructorCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetBodyDestroyCallback(Addressable listener, NewtonWorldListenerBodyDestroyCallback callback, MemorySession session) {
		MemorySegment callbackFunc = NewtonWorldListenerBodyDestroyCallback.allocate(callback, session);
		Newton_h.NewtonWorldListenerSetBodyDestroyCallback(address, listener, callbackFunc);
	}
	
	public void listenerDebug(Addressable context) {
		Newton_h.NewtonWorldListenerDebug(address, context);
	}
	
	public MemoryAddress getListenerUserData(Addressable listener) {
		return Newton_h.NewtonWorldGetListenerUserData(address, listener);
	}
	
	public NewtonWorldListenerBodyDestroyCallback listenerGetBodyDestroyCallback(Addressable listener, MemorySession session) {
		return NewtonWorldListenerBodyDestroyCallback.ofAddress(Newton_h.NewtonWorldListenerGetBodyDestroyCallback(address, listener), session);
	}
	
	public void setDestructorCallback(NewtonWorldDestructorCallback destructor, MemorySession session) {
		MemorySegment destructorFunc = NewtonWorldDestructorCallback.allocate(destructor, session);
		Newton_h.NewtonWorldSetDestructorCallback(address, destructorFunc);
	}
	
	public NewtonWorldDestructorCallback getDestructorCallback(MemorySession session) {
		return NewtonWorldDestructorCallback.ofAddress(Newton_h.NewtonWorldGetDestructorCallback(address), session);
	}
	
	public void setCollisionConstructorDestructorCallback(NewtonCollisionCopyConstructionCallback constructor, NewtonCollisionDestructorCallback destructor, MemorySession session) {
		MemorySegment constructorFunc = NewtonCollisionCopyConstructionCallback.allocate(constructor, session);
		MemorySegment destructorFunc = NewtonCollisionDestructorCallback.allocate(destructor, session);
		Newton_h.NewtonWorldSetCollisionConstructorDestructorCallback(address, constructorFunc, destructorFunc);
	}
	
	public void setCreateDestroyContactCallback(NewtonCreateContactCallback createContact, NewtonDestroyContactCallback destroyContact, MemorySession session) {
		MemorySegment createFunc = NewtonCreateContactCallback.allocate(createContact, session);
		MemorySegment destroyFunc = NewtonDestroyContactCallback.allocate(destroyContact, session);
		Newton_h.NewtonWorldSetCreateDestroyContactCallback(address, createFunc, destroyFunc);
	}
	
	public void rayCast(float[] p0, float[] p1, NewtonWorldRayFilterCallback filter, Addressable userData, NewtonWorldRayPrefilterCallback prefilter, int threadIndex,
						MemorySession session) {
		MemorySegment p0Segment = session.allocateArray(Newton_h.C_FLOAT, p0);
		MemorySegment p1Segment = session.allocateArray(Newton_h.C_FLOAT, p1);
		MemorySegment filterFunc = NewtonWorldRayFilterCallback.allocate(filter, session);
		MemorySegment preFilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, session);
		Newton_h.NewtonWorldRayCast(address, p0Segment, p1Segment, filterFunc, userData, preFilterFunc, threadIndex);
	}
	
	public int convexCast(float[] matrix, float[] target, NewtonCollision shape, MemorySegment param, Addressable userData, NewtonWorldRayPrefilterCallback prefilter,
						  MemorySegment info, int maxContactsCount, int threadIndex, MemorySession session) {
		MemorySegment matrixSegment = session.allocateArray(Newton_h.C_FLOAT, matrix);
		MemorySegment targetSegment = session.allocateArray(Newton_h.C_FLOAT, target);
		MemorySegment prefilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, session);
		return Newton_h.NewtonWorldConvexCast(address, matrixSegment, targetSegment, shape.address, param, userData, prefilterFunc, info, maxContactsCount, threadIndex);
	}
	
	public int getBodyCount() {
		return Newton_h.NewtonWorldGetBodyCount(address);
	}
	
	public int getConstraintCount() {
		return Newton_h.NewtonWorldGetConstraintCount(address);
	}
	
	public int createMaterialGroupID() {
		return Newton_h.NewtonMaterialCreateGroupID(address);
	}
	
	public int getDefaultMaterialGroupID() {
		return Newton_h.NewtonMaterialGetDefaultGroupID(address);
	}
	
	public void destroyAllMaterialGroupIDs() {
		Newton_h.NewtonMaterialDestroyAllGroupID(address);
	}
	
	public MemoryAddress getMaterialUserData(int id0, int id1) {
		return Newton_h.NewtonMaterialGetUserData(address, id0, id1);
	}
	
	public void setMaterialSurfaceThickness(int id0, int id1, float thickness) {
		Newton_h.NewtonMaterialSetSurfaceThickness(address, id0, id1, thickness);
	}
	
	public void setMaterialCallbackUserData(int id0, int id1, Addressable userData) {
		Newton_h.NewtonMaterialSetCallbackUserData(address, id0, id1, userData);
	}
	
	public void setMaterialContactGenerationCallback(int id0, int id1, NewtonOnContactGeneration contactGeneration, MemorySession session) {
		MemorySegment contactFunc = NewtonOnContactGeneration.allocate(contactGeneration, session);
		Newton_h.NewtonMaterialSetContactGenerationCallback(address, id0, id1, contactFunc);
	}
	
	public void setMaterialCompoundCollisionCallback(int id0, int id1, NewtonOnCompoundSubCollisionAABBOverlap compoundAabbOverlap, MemorySession session) {
		MemorySegment overLapFunc = NewtonOnCompoundSubCollisionAABBOverlap.allocate(compoundAabbOverlap, session);
		Newton_h.NewtonMaterialSetCompoundCollisionCallback(address, id0, id1, overLapFunc);
	}
	
	public void setMaterialCollisionCallback(int id0, int id1, NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess process, MemorySession session) {
		MemorySegment overlapFunc = NewtonOnAABBOverlap.allocate(aabbOverlap, session);
		MemorySegment processFunc = NewtonContactsProcess.allocate(process, session);
		Newton_h.NewtonMaterialSetCollisionCallback(address, id0, id1, overlapFunc, processFunc);
	}
	
	public void setMaterialDefaultSoftness(int id0, int id1, float softness) {
		Newton_h.NewtonMaterialSetDefaultSoftness(address, id0, id1, softness);
	}
	
	public void setMaterialDefaultElasticity(int id0, int id1, float elasticCoef) {
		Newton_h.NewtonMaterialSetDefaultElasticity(address, id0, id1, elasticCoef);
	}
	
	public void setMaterialDefaultCollidable(int id0, int id1, int state) {
		Newton_h.NewtonMaterialSetDefaultCollidable(address, id0, id1, state);
	}
	
	public void setMaterialDefaultFriction(int id0, int id1, float staticFriction, float kineticFriction) {
		Newton_h.NewtonMaterialSetDefaultFriction(address, id0, id1, staticFriction, kineticFriction);
	}
	
	public void resetMaterialJointIntraJointCollision(int id0, int id1) {
		Newton_h.NewtonMaterialJointResetIntraJointCollision(address, id0, id1);
	}
	
	public void resetMaterialJointSelftJointCollision(int id0, int id1) {
		Newton_h.NewtonMaterialJointResetSelftJointCollision(address, id0, id1);
	}
	
	public NewtonMaterial getFirstMaterial() {
		return new NewtonMaterial(Newton_h.NewtonWorldGetFirstMaterial(address));
	}
	
	public NewtonMaterial getNextMaterial(NewtonMaterial material) {
		MemoryAddress materialPtr = Newton_h.NewtonWorldGetNextMaterial(address, material.address);
		return materialPtr.equals(MemoryAddress.NULL) ? null : new NewtonMaterial(materialPtr);
	}
	
	public NewtonBody getFirstBody() {
		return NewtonBody.wrap(Newton_h.NewtonWorldGetFirstBody(address));
	}
	
	public NewtonBody getNextBody(NewtonBody body) {
		MemoryAddress bodyPtr = Newton_h.NewtonWorldGetNextBody(address, body.address);
		return bodyPtr.equals(MemoryAddress.NULL) ? null : NewtonBody.wrap(bodyPtr);
	}
	
	public NewtonCollision createCollisionFromSerialization(NewtonDeserializeCallback deserializeFunction, Addressable serializeHandle, MemorySession session) {
		MemorySegment deserializeFunc = NewtonDeserializeCallback.allocate(deserializeFunction, session);
		return NewtonCollision.wrap(Newton_h.NewtonCreateCollisionFromSerialization(address, deserializeFunc, serializeHandle));
	}
	
	public void serializeCollision(NewtonCollision collision, NewtonSerializeCallback serializeFunction, Addressable serializeHandle, MemorySession session) {
		MemorySegment serializeFunc = NewtonSerializeCallback.allocate(serializeFunction, session);
		Newton_h.NewtonCollisionSerialize(address, collision.address, serializeFunc, serializeHandle);
	}
	
	public void destroyAllBodies() {
		Newton_h.NewtonDestroyAllBodies(address);
	}
	/**
	 * This method wraps a memory address into a NewtonWorld object.
	 * This method is only meant to be used internally. Improper use of this method could
	 * result in errors or an exception.
	 * @param address - MemoryAddress of NewtonWorld
	 * @return NewtonWorld object
	 */
	protected static NewtonWorld wrap(MemoryAddress address) {
		return new NewtonWorld(address);
	}
	
	public record JointSerializationCallbacks(NewtonOnJointSerializationCallback serializeCallback, NewtonOnJointDeserializationCallback deserializeCallback) {}
}
