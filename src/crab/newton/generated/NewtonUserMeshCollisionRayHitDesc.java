// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonUserMeshCollisionRayHitDesc {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        MemoryLayout.sequenceLayout(4, Constants$root.C_FLOAT$LAYOUT).withName("m_p0"),
        MemoryLayout.sequenceLayout(4, Constants$root.C_FLOAT$LAYOUT).withName("m_p1"),
        MemoryLayout.sequenceLayout(4, Constants$root.C_FLOAT$LAYOUT).withName("m_normalOut"),
        Constants$root.C_LONG_LONG$LAYOUT.withName("m_userIdOut"),
        Constants$root.C_POINTER$LAYOUT.withName("m_userData")
    ).withName("NewtonUserMeshCollisionRayHitDesc");
    public static MemoryLayout $LAYOUT() {
        return NewtonUserMeshCollisionRayHitDesc.$struct$LAYOUT;
    }
    public static MemorySegment m_p0$slice(MemorySegment seg) {
        return seg.asSlice(0, 16);
    }
    public static MemorySegment m_p1$slice(MemorySegment seg) {
        return seg.asSlice(16, 16);
    }
    public static MemorySegment m_normalOut$slice(MemorySegment seg) {
        return seg.asSlice(32, 16);
    }
    static final VarHandle m_userIdOut$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_userIdOut"));
    public static VarHandle m_userIdOut$VH() {
        return NewtonUserMeshCollisionRayHitDesc.m_userIdOut$VH;
    }
    public static long m_userIdOut$get(MemorySegment seg) {
        return (long)NewtonUserMeshCollisionRayHitDesc.m_userIdOut$VH.get(seg);
    }
    public static void m_userIdOut$set( MemorySegment seg, long x) {
        NewtonUserMeshCollisionRayHitDesc.m_userIdOut$VH.set(seg, x);
    }
    public static long m_userIdOut$get(MemorySegment seg, long index) {
        return (long)NewtonUserMeshCollisionRayHitDesc.m_userIdOut$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_userIdOut$set(MemorySegment seg, long index, long x) {
        NewtonUserMeshCollisionRayHitDesc.m_userIdOut$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_userData$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_userData"));
    public static VarHandle m_userData$VH() {
        return NewtonUserMeshCollisionRayHitDesc.m_userData$VH;
    }
    public static MemoryAddress m_userData$get(MemorySegment seg) {
        return (jdk.incubator.foreign.MemoryAddress)NewtonUserMeshCollisionRayHitDesc.m_userData$VH.get(seg);
    }
    public static void m_userData$set( MemorySegment seg, MemoryAddress x) {
        NewtonUserMeshCollisionRayHitDesc.m_userData$VH.set(seg, x);
    }
    public static MemoryAddress m_userData$get(MemorySegment seg, long index) {
        return (jdk.incubator.foreign.MemoryAddress)NewtonUserMeshCollisionRayHitDesc.m_userData$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_userData$set(MemorySegment seg, long index, MemoryAddress x) {
        NewtonUserMeshCollisionRayHitDesc.m_userData$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(int len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment allocate(ResourceScope scope) { return allocate(SegmentAllocator.nativeAllocator(scope)); }
    public static MemorySegment allocateArray(int len, ResourceScope scope) {
        return allocateArray(len, SegmentAllocator.nativeAllocator(scope));
    }
    public static MemorySegment ofAddress(MemoryAddress addr, ResourceScope scope) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, scope); }
}


