using System.Runtime.InteropServices;

public static class NativeBridge
{
    [DllImport("BuoyancyDLL")]
    public static extern int CreateBuoyancyInstance(
        float[] vertices, 
        int vertexCount,
        int[] indices,
        int indexCount,
        float comX,
        float comY,
        float comZ,
        float rho, float gravity
        );

    [DllImport("BuoyancyDLL")]
    public static extern void ComputeBuoyancy(
        int handle,
        float[] transformMatrix,
        float[] vertexHeights,
        int vertexHeightsCount,
        float[] outForceTorque
        );

    [DllImport("BuoyancyDLL")]
    public static extern void DestroyBuoyancyInstance(int handle);

    [DllImport("BuoyancyDLL")]
    public static extern void ComputeBuoyancyFromTriangles(
        int handle,
        float[] worldVertices,
        int[] indices,
        int indexCount,
        float[] vertexHeights,
        float[] outForceTorque,
        float comWx, float comWy, float comWz);

}
