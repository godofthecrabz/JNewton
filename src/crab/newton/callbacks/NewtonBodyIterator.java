// Generated by jextract

package crab.newton.callbacks;

import crab.newton.internal.*;
import jdk.incubator.foreign.*;
public interface NewtonBodyIterator {

    int apply(jdk.incubator.foreign.MemoryAddress x0, jdk.incubator.foreign.MemoryAddress x1);
    static NativeSymbol allocate(NewtonBodyIterator fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonBodyIterator.class, fi, constants$14.NewtonBodyIterator$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;Ljdk/incubator/foreign/MemoryAddress;)I", scope);
    }
    static NewtonBodyIterator ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonBodyIterator::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0, jdk.incubator.foreign.MemoryAddress x1) -> {
            try {
                return (int)constants$14.NewtonBodyIterator$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0, (jdk.incubator.foreign.Addressable)x1);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


