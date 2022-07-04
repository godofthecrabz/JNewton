// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonCollisionMaterial {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_LONG_LONG$LAYOUT.withName("m_userId"),
        MemoryLayout.unionLayout(
            Constants$root.C_POINTER$LAYOUT.withName("m_ptr"),
            Constants$root.C_LONG_LONG$LAYOUT.withName("m_int"),
            Constants$root.C_FLOAT$LAYOUT.withName("m_float")
        ).withName("m_userData"),
        MemoryLayout.sequenceLayout(6, MemoryLayout.unionLayout(
            Constants$root.C_POINTER$LAYOUT.withName("m_ptr"),
            Constants$root.C_LONG_LONG$LAYOUT.withName("m_int"),
            Constants$root.C_FLOAT$LAYOUT.withName("m_float")
        )).withName("m_userParam")
    ).withName("NewtonCollisionMaterial");
    public static MemoryLayout $LAYOUT() {
        return NewtonCollisionMaterial.$struct$LAYOUT;
    }
    static final VarHandle m_userId$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_userId"));
    public static VarHandle m_userId$VH() {
        return NewtonCollisionMaterial.m_userId$VH;
    }
    public static long m_userId$get(MemorySegment seg) {
        return (long)NewtonCollisionMaterial.m_userId$VH.get(seg);
    }
    public static void m_userId$set( MemorySegment seg, long x) {
        NewtonCollisionMaterial.m_userId$VH.set(seg, x);
    }
    public static long m_userId$get(MemorySegment seg, long index) {
        return (long)NewtonCollisionMaterial.m_userId$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_userId$set(MemorySegment seg, long index, long x) {
        NewtonCollisionMaterial.m_userId$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static MemorySegment m_userData$slice(MemorySegment seg) {
        return seg.asSlice(8, 8);
    }
    public static MemorySegment m_userParam$slice(MemorySegment seg) {
        return seg.asSlice(16, 48);
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


