  #ifdef _WIN32
      #ifdef BUOYANCY_EXPORTS
          #define BUOYANCY_API __declspec(dllexport)
      #else
          #define BUOYANCY_API __declspec(dllimport)
      #endif
  #else
      #define BUOYANCY_API
  #endif

extern "C" {
    BUOYANCY_API int CreateBuoyancyInstance(
        const float* vertices, int vertexCount,
        const int* indices, int indexCount,
        float comX, float comY, float comZ,
        float rho, float gravity);

    BUOYANCY_API void ComputeBuoyancy(
        int handle,
        const float* transformMatrix,
        const float* vertexHeights,
        int vertexHeightsCount,
        float* outForceTorque);

    BUOYANCY_API void ComputeBuoyancyFromTriangles(
        int handle,
        const float* worldVertices,
        const int* indices, 
        int indexCount,
        const float* vertexHeights,
        float* outForceTorque,
        float comWx, float comWy, float comWz);

    BUOYANCY_API void DestroyBuoyancyInstance(int handle);
}