// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$6 {

    static final FunctionDescriptor NewtonUserMeshCollisionDestroyCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUserMeshCollisionDestroyCallback$MH = RuntimeHelper.downcallHandle(
        constants$6.NewtonUserMeshCollisionDestroyCallback$FUNC, false
    );
    static final FunctionDescriptor NewtonUserMeshCollisionRayHitCallback$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUserMeshCollisionRayHitCallback$MH = RuntimeHelper.downcallHandle(
        constants$6.NewtonUserMeshCollisionRayHitCallback$FUNC, false
    );
    static final FunctionDescriptor NewtonUserMeshCollisionGetCollisionInfo$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUserMeshCollisionGetCollisionInfo$MH = RuntimeHelper.downcallHandle(
        constants$6.NewtonUserMeshCollisionGetCollisionInfo$FUNC, false
    );
}


