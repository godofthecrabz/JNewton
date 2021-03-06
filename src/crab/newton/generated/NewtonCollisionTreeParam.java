// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonCollisionTreeParam {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_LONG$LAYOUT.withName("m_vertexCount"),
        Constants$root.C_LONG$LAYOUT.withName("m_indexCount")
    ).withName("NewtonCollisionTreeParam");
    public static MemoryLayout $LAYOUT() {
        return NewtonCollisionTreeParam.$struct$LAYOUT;
    }
    static final VarHandle m_vertexCount$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_vertexCount"));
    public static VarHandle m_vertexCount$VH() {
        return NewtonCollisionTreeParam.m_vertexCount$VH;
    }
    public static int m_vertexCount$get(MemorySegment seg) {
        return (int)NewtonCollisionTreeParam.m_vertexCount$VH.get(seg);
    }
    public static void m_vertexCount$set( MemorySegment seg, int x) {
        NewtonCollisionTreeParam.m_vertexCount$VH.set(seg, x);
    }
    public static int m_vertexCount$get(MemorySegment seg, long index) {
        return (int)NewtonCollisionTreeParam.m_vertexCount$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_vertexCount$set(MemorySegment seg, long index, int x) {
        NewtonCollisionTreeParam.m_vertexCount$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_indexCount$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_indexCount"));
    public static VarHandle m_indexCount$VH() {
        return NewtonCollisionTreeParam.m_indexCount$VH;
    }
    public static int m_indexCount$get(MemorySegment seg) {
        return (int)NewtonCollisionTreeParam.m_indexCount$VH.get(seg);
    }
    public static void m_indexCount$set( MemorySegment seg, int x) {
        NewtonCollisionTreeParam.m_indexCount$VH.set(seg, x);
    }
    public static int m_indexCount$get(MemorySegment seg, long index) {
        return (int)NewtonCollisionTreeParam.m_indexCount$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_indexCount$set(MemorySegment seg, long index, int x) {
        NewtonCollisionTreeParam.m_indexCount$VH.set(seg.asSlice(index*sizeof()), x);
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


