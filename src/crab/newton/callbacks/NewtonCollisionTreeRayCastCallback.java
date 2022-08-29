// Generated by jextract

package crab.newton.callbacks;

import crab.newton.internal.*;
import jdk.incubator.foreign.*;
public interface NewtonCollisionTreeRayCastCallback {

    float apply(jdk.incubator.foreign.MemoryAddress x0, jdk.incubator.foreign.MemoryAddress x1, float x2, jdk.incubator.foreign.MemoryAddress x3, int x4, jdk.incubator.foreign.MemoryAddress x5);
    static NativeSymbol allocate(NewtonCollisionTreeRayCastCallback fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonCollisionTreeRayCastCallback.class, fi, constants$8.NewtonCollisionTreeRayCastCallback$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;Ljdk/incubator/foreign/MemoryAddress;FLjdk/incubator/foreign/MemoryAddress;ILjdk/incubator/foreign/MemoryAddress;)F", scope);
    }
    static NewtonCollisionTreeRayCastCallback ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonCollisionTreeRayCastCallback::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0, jdk.incubator.foreign.MemoryAddress x1, float x2, jdk.incubator.foreign.MemoryAddress x3, int x4, jdk.incubator.foreign.MemoryAddress x5) -> {
            try {
                return (float)constants$8.NewtonCollisionTreeRayCastCallback$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0, (jdk.incubator.foreign.Addressable)x1, x2, (jdk.incubator.foreign.Addressable)x3, x4, (jdk.incubator.foreign.Addressable)x5);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

