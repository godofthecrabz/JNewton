// Generated by jextract

package crab.newton;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;

import crab.newton.internal.*;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonHingeSliderUpdateDesc {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_FLOAT$LAYOUT.withName("m_accel"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_minFriction"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_maxFriction"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_timestep")
    ).withName("NewtonHingeSliderUpdateDesc");
    public static MemoryLayout $LAYOUT() {
        return NewtonHingeSliderUpdateDesc.$struct$LAYOUT;
    }
    static final VarHandle m_accel$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_accel"));
    public static VarHandle m_accel$VH() {
        return NewtonHingeSliderUpdateDesc.m_accel$VH;
    }
    public static float m_accel$get(MemorySegment seg) {
        return (float)NewtonHingeSliderUpdateDesc.m_accel$VH.get(seg);
    }
    public static void m_accel$set( MemorySegment seg, float x) {
        NewtonHingeSliderUpdateDesc.m_accel$VH.set(seg, x);
    }
    public static float m_accel$get(MemorySegment seg, long index) {
        return (float)NewtonHingeSliderUpdateDesc.m_accel$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_accel$set(MemorySegment seg, long index, float x) {
        NewtonHingeSliderUpdateDesc.m_accel$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_minFriction$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_minFriction"));
    public static VarHandle m_minFriction$VH() {
        return NewtonHingeSliderUpdateDesc.m_minFriction$VH;
    }
    public static float m_minFriction$get(MemorySegment seg) {
        return (float)NewtonHingeSliderUpdateDesc.m_minFriction$VH.get(seg);
    }
    public static void m_minFriction$set( MemorySegment seg, float x) {
        NewtonHingeSliderUpdateDesc.m_minFriction$VH.set(seg, x);
    }
    public static float m_minFriction$get(MemorySegment seg, long index) {
        return (float)NewtonHingeSliderUpdateDesc.m_minFriction$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_minFriction$set(MemorySegment seg, long index, float x) {
        NewtonHingeSliderUpdateDesc.m_minFriction$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_maxFriction$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_maxFriction"));
    public static VarHandle m_maxFriction$VH() {
        return NewtonHingeSliderUpdateDesc.m_maxFriction$VH;
    }
    public static float m_maxFriction$get(MemorySegment seg) {
        return (float)NewtonHingeSliderUpdateDesc.m_maxFriction$VH.get(seg);
    }
    public static void m_maxFriction$set( MemorySegment seg, float x) {
        NewtonHingeSliderUpdateDesc.m_maxFriction$VH.set(seg, x);
    }
    public static float m_maxFriction$get(MemorySegment seg, long index) {
        return (float)NewtonHingeSliderUpdateDesc.m_maxFriction$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_maxFriction$set(MemorySegment seg, long index, float x) {
        NewtonHingeSliderUpdateDesc.m_maxFriction$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_timestep$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_timestep"));
    public static VarHandle m_timestep$VH() {
        return NewtonHingeSliderUpdateDesc.m_timestep$VH;
    }
    public static float m_timestep$get(MemorySegment seg) {
        return (float)NewtonHingeSliderUpdateDesc.m_timestep$VH.get(seg);
    }
    public static void m_timestep$set( MemorySegment seg, float x) {
        NewtonHingeSliderUpdateDesc.m_timestep$VH.set(seg, x);
    }
    public static float m_timestep$get(MemorySegment seg, long index) {
        return (float)NewtonHingeSliderUpdateDesc.m_timestep$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_timestep$set(MemorySegment seg, long index, float x) {
        NewtonHingeSliderUpdateDesc.m_timestep$VH.set(seg.asSlice(index*sizeof()), x);
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


