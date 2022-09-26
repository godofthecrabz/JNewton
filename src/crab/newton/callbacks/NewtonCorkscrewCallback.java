package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonCorkscrewCallback {

    int apply(java.lang.foreign.MemoryAddress corkscrew, java.lang.foreign.MemoryAddress desc);
    static MemorySegment allocate(NewtonCorkscrewCallback fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonCorkscrewCallback.class, fi, constants$16.NewtonCorkscrewCallback$FUNC, session);
    }
    static NewtonCorkscrewCallback ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _corkscrew, java.lang.foreign.MemoryAddress _desc) -> {
            try {
                return (int)constants$16.NewtonCorkscrewCallback$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_corkscrew, (java.lang.foreign.Addressable)_desc);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


