// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$55 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$55() {}
    public static final FunctionDescriptor NewtonCollisionDataPointer$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCollisionDataPointer$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionDataPointer",
        constants$55.NewtonCollisionDataPointer$FUNC
    );
    public static final FunctionDescriptor NewtonCreateCompoundCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonCreateCompoundCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateCompoundCollision",
        constants$55.NewtonCreateCompoundCollision$FUNC
    );
    public static final FunctionDescriptor NewtonCreateCompoundCollisionFromMesh$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonCreateCompoundCollisionFromMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateCompoundCollisionFromMesh",
        constants$55.NewtonCreateCompoundCollisionFromMesh$FUNC
    );
    public static final FunctionDescriptor NewtonCompoundCollisionBeginAddRemove$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCompoundCollisionBeginAddRemove$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionBeginAddRemove",
        constants$55.NewtonCompoundCollisionBeginAddRemove$FUNC
    );
    public static final FunctionDescriptor NewtonCompoundCollisionAddSubCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCompoundCollisionAddSubCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionAddSubCollision",
        constants$55.NewtonCompoundCollisionAddSubCollision$FUNC
    );
    public static final FunctionDescriptor NewtonCompoundCollisionRemoveSubCollision$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCompoundCollisionRemoveSubCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionRemoveSubCollision",
        constants$55.NewtonCompoundCollisionRemoveSubCollision$FUNC
    );
}


