// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$47 {

    static final FunctionDescriptor NewtonFracturedCompoundMeshPartGetIndexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonFracturedCompoundMeshPartGetIndexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundMeshPartGetIndexCount",
        constants$47.NewtonFracturedCompoundMeshPartGetIndexCount$FUNC, false
    );
    static final FunctionDescriptor NewtonCreateSceneCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonCreateSceneCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateSceneCollision",
        constants$47.NewtonCreateSceneCollision$FUNC, false
    );
    static final FunctionDescriptor NewtonSceneCollisionBeginAddRemove$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSceneCollisionBeginAddRemove$MH = RuntimeHelper.downcallHandle(
        "NewtonSceneCollisionBeginAddRemove",
        constants$47.NewtonSceneCollisionBeginAddRemove$FUNC, false
    );
    static final FunctionDescriptor NewtonSceneCollisionAddSubCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSceneCollisionAddSubCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonSceneCollisionAddSubCollision",
        constants$47.NewtonSceneCollisionAddSubCollision$FUNC, false
    );
    static final FunctionDescriptor NewtonSceneCollisionRemoveSubCollision$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSceneCollisionRemoveSubCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonSceneCollisionRemoveSubCollision",
        constants$47.NewtonSceneCollisionRemoveSubCollision$FUNC, false
    );
    static final FunctionDescriptor NewtonSceneCollisionRemoveSubCollisionByIndex$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonSceneCollisionRemoveSubCollisionByIndex$MH = RuntimeHelper.downcallHandle(
        "NewtonSceneCollisionRemoveSubCollisionByIndex",
        constants$47.NewtonSceneCollisionRemoveSubCollisionByIndex$FUNC, false
    );
}


