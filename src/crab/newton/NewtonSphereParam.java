package crab.newton;// Generated by jextract

import java.lang.invoke.VarHandle;
import java.lang.foreign.*;
import crab.newton.internal.*;

public class NewtonSphereParam {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_FLOAT$LAYOUT.withName("m_radio")
    ).withName("crab.newton.NewtonSphereParam");
    public static MemoryLayout $LAYOUT() {
        return NewtonSphereParam.$struct$LAYOUT;
    }
    static final VarHandle m_radio$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_radio"));
    public static VarHandle m_radio$VH() {
        return NewtonSphereParam.m_radio$VH;
    }
    public static float m_radio$get(MemorySegment seg) {
        return (float)NewtonSphereParam.m_radio$VH.get(seg);
    }
    public static void m_radio$set( MemorySegment seg, float x) {
        NewtonSphereParam.m_radio$VH.set(seg, x);
    }
    public static float m_radio$get(MemorySegment seg, long index) {
        return (float)NewtonSphereParam.m_radio$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_radio$set(MemorySegment seg, long index, float x) {
        NewtonSphereParam.m_radio$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(int len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment ofAddress(MemoryAddress addr, MemorySession session) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, session); }
}


