// Generated by jextract

package crab.newton.callbacks;

import crab.newton.internal.*;
import jdk.incubator.foreign.*;
public interface NewtonOnUserCollisionSerializationCallback {

    void apply(jdk.incubator.foreign.MemoryAddress x0, jdk.incubator.foreign.MemoryAddress x1, jdk.incubator.foreign.MemoryAddress x2);
    static NativeSymbol allocate(NewtonOnUserCollisionSerializationCallback fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonOnUserCollisionSerializationCallback.class, fi, constants$5.NewtonOnUserCollisionSerializationCallback$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;Ljdk/incubator/foreign/MemoryAddress;Ljdk/incubator/foreign/MemoryAddress;)V", scope);
    }
    static NewtonOnUserCollisionSerializationCallback ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonOnUserCollisionSerializationCallback::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0, jdk.incubator.foreign.MemoryAddress x1, jdk.incubator.foreign.MemoryAddress x2) -> {
            try {
                constants$5.NewtonOnUserCollisionSerializationCallback$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0, (jdk.incubator.foreign.Addressable)x1, (jdk.incubator.foreign.Addressable)x2);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

