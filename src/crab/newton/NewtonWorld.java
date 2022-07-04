package crab.newton;

import jdk.incubator.foreign.*;

import crab.newton.generated.*;

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
	 * @param scope - ResourceScope for allocating upcall stubs
	 */
	public static void setMemorySystem(NewtonAllocMemory alloc, NewtonFreeMemory free, ResourceScope scope) {
		NativeSymbol allocFunc = NewtonAllocMemory.allocate(alloc, scope);
		NativeSymbol freeFunc = NewtonFreeMemory.allocate(free, scope);
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
	 * @param scope - ResourceScope for the returned MemorySegment
	 * @return MemorySegment representing the allocated memory
	 */
	public static MemorySegment newtonAlloc(int sizeInBytes, ResourceScope scope) {
		return MemorySegment.ofAddress(Newton_h.NewtonAlloc(sizeInBytes), sizeInBytes, scope);
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
	 * @param scope - ResourceScope for the returned MemorySegment
	 * @return MemorySegment representing the allocated memory
	 */
	public static MemorySegment newtonAlloc(MemoryLayout layout, ResourceScope scope) {
		return MemorySegment.ofAddress(Newton_h.NewtonAlloc((int) layout.byteSize()), layout.byteSize(), scope);
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
	
	public NewtonPostUpdateCallback getPostUpdateCallback(ResourceScope scope) {
		return NewtonPostUpdateCallback.ofAddress(Newton_h.NewtonGetPostUpdateCallback(address), scope);
	}
	
	public void setPostUpdateCallback(NewtonPostUpdateCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonPostUpdateCallback.allocate(callback, scope);
		Newton_h.NewtonSetPostUpdateCallback(address, callbackFunc);
	}
	
	public void loadPlugins(String pluginPath) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment path = allocator.allocateUtf8String(pluginPath);
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
			ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment filePath = allocator.allocateUtf8String(filename);
		NativeSymbol callbackFunc = NewtonOnBodySerializationCallback.allocate(bodyCallback, scope);
		Newton_h.NewtonSerializeToFile(address, filePath, callbackFunc, bodyUserData);
	}
	
	public void deserializeFromFile(String filename, NewtonOnBodyDeserializationCallback bodyCallback, Addressable bodyUserData,
			ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment filePath = allocator.allocateUtf8String(filename);
		NativeSymbol callbackFunc = NewtonOnBodyDeserializationCallback.allocate(bodyCallback, scope);
		Newton_h.NewtonDeserializeFromFile(address, filePath, callbackFunc, bodyUserData);
	}
	
	public void serializeScene(NewtonOnBodySerializationCallback bodyCallback, Addressable bodyUserData, NewtonSerializeCallback serializeCallback, Addressable serializeHandle,
			ResourceScope scope) {
		NativeSymbol bodyFunc = NewtonOnBodySerializationCallback.allocate(bodyCallback, scope);
		NativeSymbol serializeFunc = NewtonSerializeCallback.allocate(serializeCallback, scope);
		Newton_h.NewtonSerializeScene(address, bodyFunc, bodyUserData, serializeFunc, serializeHandle);
	}
	
	public void deserializeScene(NewtonOnBodyDeserializationCallback bodyCallback, Addressable bodyUserData, NewtonDeserializeCallback serializeCallback, Addressable serializeHandle,
			ResourceScope scope) {
		NativeSymbol bodyFunc = NewtonOnBodyDeserializationCallback.allocate(bodyCallback, scope);
		NativeSymbol deserializeFunc = NewtonDeserializeCallback.allocate(serializeCallback, scope);
		Newton_h.NewtonDeserializeScene(address, bodyFunc, bodyUserData, deserializeFunc, serializeHandle);
	}
	
	public NewtonBody findSerializedBody(int bodySerializedID) {
		MemoryAddress body = Newton_h.NewtonFindSerializedBody(address, bodySerializedID);
		int bodyType = Newton_h.NewtonBodyGetType(body);
		return switch (bodyType) {
			case 0 -> new NewtonDynamicBody(body);
			case 1 -> new NewtonKinematicBody(body);
			default -> throw new RuntimeException("Error finding serialized body");
		};
	}
	
	public void setJointSerializationCallbacks(NewtonOnJointSerializationCallback serializeJoint, NewtonOnJointDeserializationCallback deserializeJoint,
			ResourceScope scope) {
		NativeSymbol serializeFunc = NewtonOnJointSerializationCallback.allocate(serializeJoint, scope);
		NativeSymbol deserializeFunc = NewtonOnJointDeserializationCallback.allocate(deserializeJoint, scope);
		Newton_h.NewtonSetJointSerializationCallbacks(address, serializeFunc, deserializeFunc);
	}
	
	public JointSerializationCallbacks getJointSerializationCallbacks(ResourceScope scope) {
		MemoryAddress serializePtr = MemoryAddress.NULL;
		MemoryAddress deserializePtr = MemoryAddress.NULL;
		Newton_h.NewtonGetJointSerializationCallbacks(address, serializePtr, deserializePtr);
		NewtonOnJointSerializationCallback serializeCallback = NewtonOnJointSerializationCallback.ofAddress(serializePtr, scope);
		NewtonOnJointDeserializationCallback deserializeCallback = NewtonOnJointDeserializationCallback.ofAddress(deserializePtr, scope);
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
	
	public void dispatchThreadJob(NewtonJobTask task, Addressable userData, String functionName, ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment function_name = allocator.allocateUtf8String(functionName);
		NativeSymbol taskFunc = NewtonJobTask.allocate(task, scope);
		Newton_h.NewtonDispachThreadJob(address, taskFunc, userData, function_name);
	}
	
	public void syncThreadJobs() {
		Newton_h.NewtonSyncThreadJobs(address);
	}
	
	public void setIslandUpdateEvent(NewtonIslandUpdate islandUpdate, ResourceScope scope) {
		NativeSymbol islandUpdateFunc = NewtonIslandUpdate.allocate(islandUpdate, scope);
		Newton_h.NewtonSetIslandUpdateEvent(address, islandUpdateFunc);
	}
	
	public void forEachJoint(NewtonJointIterator callback, Addressable userData, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonJointIterator.allocate(callback, scope);
		Newton_h.NewtonWorldForEachJointDo(address, callbackFunc, userData);
	}
	
	public void forEachBodyInAABB(float[] p0, float[] p1, NewtonBodyIterator callback, Addressable userData, ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment p0Segment = allocator.allocateArray(Newton_h.C_FLOAT, p0);
		MemorySegment p1Segment = allocator.allocateArray(Newton_h.C_FLOAT, p1);
		NativeSymbol callbackFunc = NewtonBodyIterator.allocate(callback, scope);
		Newton_h.NewtonWorldForEachBodyInAABBDo(address, p0Segment, p1Segment, callbackFunc, userData);
	}
	
	public void setUserData(Addressable userData) {
		Newton_h.NewtonWorldSetUserData(address, userData);
	}
	
	public MemoryAddress getUserData() {
		return Newton_h.NewtonWorldGetUserData(address);
	}
	
	public MemoryAddress addListener(String nameId, Addressable listenerUserData, ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment nameIdSegment = allocator.allocateUtf8String(nameId);
		return Newton_h.NewtonWorldAddListener(address, nameIdSegment, listenerUserData);
	}
	
	public MemoryAddress getListener(String nameId, ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment nameIdSegment = allocator.allocateUtf8String(nameId);
		return Newton_h.NewtonWorldGetListener(address, nameIdSegment);
	}
	
	public void listenerSetDebugCallback(Addressable listener, NewtonWorldListenerDebugCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonWorldListenerDebugCallback.allocate(callback, scope);
		Newton_h.NewtonWorldListenerSetDebugCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetPostStepCallback(Addressable listener, NewtonWorldUpdateListenerCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, scope);
		Newton_h.NewtonWorldListenerSetPostStepCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetPreUpdateCallback(Addressable listener, NewtonWorldUpdateListenerCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, scope);
		Newton_h.NewtonWorldListenerSetPreUpdateCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetPostUpdateCallback(Addressable listener, NewtonWorldUpdateListenerCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, scope);
		Newton_h.NewtonWorldListenerSetPostUpdateCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetDestructorCallback(Addressable listener, NewtonWorldDestroyListenerCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonWorldDestroyListenerCallback.allocate(callback, scope);
		Newton_h.NewtonWorldListenerSetDestructorCallback(address, listener, callbackFunc);
	}
	
	public void listenerSetBodyDestroyCallback(Addressable listener, NewtonWorldListenerBodyDestroyCallback callback, ResourceScope scope) {
		NativeSymbol callbackFunc = NewtonWorldListenerBodyDestroyCallback.allocate(callback, scope);
		Newton_h.NewtonWorldListenerSetBodyDestroyCallback(address, listener, callbackFunc);
	}
	
	public void listenerDebug(Addressable context) {
		Newton_h.NewtonWorldListenerDebug(address, context);
	}
	
	public MemoryAddress getListenerUserData(Addressable listener) {
		return Newton_h.NewtonWorldGetListenerUserData(address, listener);
	}
	
	public NewtonWorldListenerBodyDestroyCallback listenerGetBodyDestroyCallback(Addressable listener, ResourceScope scope) {
		return NewtonWorldListenerBodyDestroyCallback.ofAddress(Newton_h.NewtonWorldListenerGetBodyDestroyCallback(address, listener), scope);
	}
	
	public void setDestructorCallback(NewtonWorldDestructorCallback destructor, ResourceScope scope) {
		NativeSymbol destructorFunc = NewtonWorldDestructorCallback.allocate(destructor, scope);
		Newton_h.NewtonWorldSetDestructorCallback(address, destructorFunc);
	}
	
	public NewtonWorldDestructorCallback getDestructorCallback(ResourceScope scope) {
		return NewtonWorldDestructorCallback.ofAddress(Newton_h.NewtonWorldGetDestructorCallback(address), scope);
	}
	
	public void setCollisionConstructorDestructorCallback(NewtonCollisionCopyConstructionCallback constructor, NewtonCollisionDestructorCallback destructor, ResourceScope scope) {
		NativeSymbol constructorFunc = NewtonCollisionCopyConstructionCallback.allocate(constructor, scope);
		NativeSymbol destructorFunc = NewtonCollisionDestructorCallback.allocate(destructor, scope);
		Newton_h.NewtonWorldSetCollisionConstructorDestructorCallback(address, constructorFunc, destructorFunc);
	}
	
	public void setCreateDestroyContactCallback(NewtonCreateContactCallback createContact, NewtonDestroyContactCallback destroyContact, ResourceScope scope) {
		NativeSymbol createFunc = NewtonCreateContactCallback.allocate(createContact, scope);
		NativeSymbol destroyFunc = NewtonDestroyContactCallback.allocate(destroyContact, scope);
		Newton_h.NewtonWorldSetCreateDestroyContactCallback(address, createFunc, destroyFunc);
	}
	
	public void rayCast(float[] p0, float[] p1, NewtonWorldRayFilterCallback filter, Addressable userData, NewtonWorldRayPrefilterCallback prefilter, int threadIndex,
			ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment p0Segment = allocator.allocateArray(Newton_h.C_FLOAT, p0);
		MemorySegment p1Segment = allocator.allocateArray(Newton_h.C_FLOAT, p1);
		NativeSymbol filterFunc = NewtonWorldRayFilterCallback.allocate(filter, scope);
		NativeSymbol preFilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, scope);
		Newton_h.NewtonWorldRayCast(address, p0Segment, p1Segment, filterFunc, userData, preFilterFunc, threadIndex);
	}
	
	public int convexCast(float[] matrix, float[] target, NewtonCollision shape, MemorySegment param, Addressable userData, NewtonWorldRayPrefilterCallback prefilter,
			MemorySegment info, int maxContactsCount, int threadIndex, ResourceScope scope) {
		SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
		MemorySegment matrixSegment = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
		MemorySegment targetSegment = allocator.allocateArray(Newton_h.C_FLOAT, target);
		NativeSymbol prefilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, scope);
		return Newton_h.NewtonWorldConvexCast(address, matrixSegment, targetSegment, shape.address(), param, userData, prefilterFunc, info, maxContactsCount, threadIndex);
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
	
	public void setMaterialContactGenerationCallback(int id0, int id1, NewtonOnContactGeneration contactGeneration, ResourceScope scope) {
		NativeSymbol contactFunc = NewtonOnContactGeneration.allocate(contactGeneration, scope);
		Newton_h.NewtonMaterialSetContactGenerationCallback(address, id0, id1, contactFunc);
	}
	
	public void setMaterialCompoundCollisionCallback(int id0, int id1, NewtonOnCompoundSubCollisionAABBOverlap compoundAabbOverlap, ResourceScope scope) {
		NativeSymbol overLapFunc = NewtonOnCompoundSubCollisionAABBOverlap.allocate(compoundAabbOverlap, scope);
		Newton_h.NewtonMaterialSetCompoundCollisionCallback(address, id0, id1, overLapFunc);
	}
	
	public void setMaterialCollisionCallback(int id0, int id1, NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess process, ResourceScope scope) {
		NativeSymbol overlapFunc = NewtonOnAABBOverlap.allocate(aabbOverlap, scope);
		NativeSymbol processFunc = NewtonContactsProcess.allocate(process, scope);
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
		MemoryAddress bodyPtr = Newton_h.NewtonWorldGetNextBody(address, body.address());
		return bodyPtr.equals(MemoryAddress.NULL) ? null : NewtonBody.wrap(bodyPtr);
	}
	
	public NewtonCollision createCollisionFromSerialization(NewtonDeserializeCallback deserializeFunction, Addressable serializeHandle, ResourceScope scope) {
		NativeSymbol deserializeFunc = NewtonDeserializeCallback.allocate(deserializeFunction, scope);
		return NewtonCollision.wrap(Newton_h.NewtonCreateCollisionFromSerialization(address, deserializeFunc, serializeHandle));
	}
	
	public void serializeCollision(NewtonCollision collision, NewtonSerializeCallback serializeFunction, Addressable serializeHandle, ResourceScope scope) {
		NativeSymbol serializeFunc = NewtonSerializeCallback.allocate(serializeFunction, scope);
		Newton_h.NewtonCollisionSerialize(address, collision.address(), serializeFunc, serializeHandle);
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
