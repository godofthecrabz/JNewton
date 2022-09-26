package crab.newton;// Generated by jextract

import java.lang.invoke.VarHandle;
import java.lang.foreign.*;
import crab.newton.internal.*;

public class NewtonCapsuleParam {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_FLOAT$LAYOUT.withName("m_radio0"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_radio1"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_height")
    ).withName("crab.newton.NewtonCapsuleParam");
    public static MemoryLayout $LAYOUT() {
        return NewtonCapsuleParam.$struct$LAYOUT;
    }
    static final VarHandle m_radio0$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_radio0"));
    public static VarHandle m_radio0$VH() {
        return NewtonCapsuleParam.m_radio0$VH;
    }
    public static float m_radio0$get(MemorySegment seg) {
        return (float)NewtonCapsuleParam.m_radio0$VH.get(seg);
    }
    public static void m_radio0$set( MemorySegment seg, float x) {
        NewtonCapsuleParam.m_radio0$VH.set(seg, x);
    }
    public static float m_radio0$get(MemorySegment seg, long index) {
        return (float)NewtonCapsuleParam.m_radio0$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_radio0$set(MemorySegment seg, long index, float x) {
        NewtonCapsuleParam.m_radio0$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_radio1$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_radio1"));
    public static VarHandle m_radio1$VH() {
        return NewtonCapsuleParam.m_radio1$VH;
    }
    public static float m_radio1$get(MemorySegment seg) {
        return (float)NewtonCapsuleParam.m_radio1$VH.get(seg);
    }
    public static void m_radio1$set( MemorySegment seg, float x) {
        NewtonCapsuleParam.m_radio1$VH.set(seg, x);
    }
    public static float m_radio1$get(MemorySegment seg, long index) {
        return (float)NewtonCapsuleParam.m_radio1$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_radio1$set(MemorySegment seg, long index, float x) {
        NewtonCapsuleParam.m_radio1$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_height$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_height"));
    public static VarHandle m_height$VH() {
        return NewtonCapsuleParam.m_height$VH;
    }
    public static float m_height$get(MemorySegment seg) {
        return (float)NewtonCapsuleParam.m_height$VH.get(seg);
    }
    public static void m_height$set( MemorySegment seg, float x) {
        NewtonCapsuleParam.m_height$VH.set(seg, x);
    }
    public static float m_height$get(MemorySegment seg, long index) {
        return (float)NewtonCapsuleParam.m_height$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_height$set(MemorySegment seg, long index, float x) {
        NewtonCapsuleParam.m_height$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(int len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment ofAddress(MemoryAddress addr, MemorySession session) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, session); }
}


