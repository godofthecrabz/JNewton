package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonSliderCallback {

    int apply(java.lang.foreign.MemoryAddress slider, java.lang.foreign.MemoryAddress desc);
    static MemorySegment allocate(NewtonSliderCallback fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonSliderCallback.class, fi, constants$16.NewtonSliderCallback$FUNC, session);
    }
    static NewtonSliderCallback ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _slider, java.lang.foreign.MemoryAddress _desc) -> {
            try {
                return (int)constants$16.NewtonSliderCallback$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_slider, (java.lang.foreign.Addressable)_desc);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


