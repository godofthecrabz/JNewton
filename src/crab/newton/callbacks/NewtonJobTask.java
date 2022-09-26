package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonJobTask {

    void apply(java.lang.foreign.MemoryAddress world, java.lang.foreign.MemoryAddress userData, int threadIndex);
    static MemorySegment allocate(NewtonJobTask fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonJobTask.class, fi, constants$18.NewtonJobTask$FUNC, session);
    }
    static NewtonJobTask ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _world, java.lang.foreign.MemoryAddress _userData, int _threadIndex) -> {
            try {
                constants$18.NewtonJobTask$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_world, (java.lang.foreign.Addressable)_userData, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


