#include "buoyancy_api.h"
#include <unordered_map>
#include <glm/glm.hpp>
#include <cstring>
#include <glm/gtc/type_ptr.hpp>

struct ObjectData
{
    glm::vec3* vertices;
    int vertexCount;
    int* indices;
    int indexCount;
    glm::vec3 com;
    glm::vec3* transformedVertices;
    float rho;
    float gravity;
};

int checkUnderWater(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float ha, float hb, float hc);
void accumulateBuoyancy(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& com, float ha, float hb, float hc, float rho, float gravity, glm::vec3& totalForce, glm::vec3& totalTorque);
void oneAboveSplit(const glm::vec3& above, const glm::vec3& first, const glm::vec3& second, float hAbove, float hFirst, float hSecond, const glm::vec3& com, float rho, float gravity, glm::vec3& totalForce, glm::vec3& totalTorque);
void oneBelowSplit(const glm::vec3& below, const glm::vec3& first, const glm::vec3& second, float hBelow, float hFirst, float hSecond, const glm::vec3& com, float rho, float gravity, glm::vec3& totalForce, glm::vec3& totalTorque);

std::unordered_map<int, ObjectData*> objects;

static int nextHandle = 0;
const float EPS = 1e-4f;

extern "C" { // To not mangle names.

    BUOYANCY_API int CreateBuoyancyInstance(
        const float* vertices, int vertexCount,
        const int* indices, int indexCount,
        float comX, float comY, float comZ,
        float rho, float gravity)
    {
        // Alloc obj 
        ObjectData* obj = new ObjectData{};
        obj->indexCount = indexCount;
        obj->vertexCount = vertexCount;
        
        // Alloc and copy C# float[] to glm::vec3[]
        obj->vertices = new glm::vec3[vertexCount];
        for (size_t i = 0; i < vertexCount; i++) {
            obj->vertices[i] = glm::vec3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        }
        
        // Alloc and copy index data
        obj->indices = new int[indexCount];
        memcpy(obj->indices, indices, indexCount * sizeof(int));

        //COM
        obj->com = glm::vec3(comX, comY, comZ);
        // Buffer for verticies
        obj->transformedVertices = new glm::vec3[vertexCount];
        //Other relevant data
        obj->rho = rho;
        obj->gravity = gravity;

        int handle = nextHandle;
        objects[handle] = obj;
        nextHandle += 1;
        return handle;
    }

    BUOYANCY_API void ComputeBuoyancyFromTriangles(
        int handle,
        const float* worldVertices,
        const int* indices,
        int indexCount,
        const float* vertexHeights,
        float* outForceTorque,
        float comWx, float comWy, float comWz)
    {
        auto it = objects.find(handle);
        if (it == objects.end()) return;
        ObjectData* obj = it->second;

        glm::vec3 totalForce(0.0f);
        glm::vec3 totalTorque(0.0f);

        glm::vec3 comW(comWx, comWy, comWz);

        for (size_t i = 0; i < indexCount; i += 3)
        {
            int ia = indices[i];
            int ib = indices[i + 1];
            int ic = indices[i + 2];

            glm::vec3 a(worldVertices[ia * 3], worldVertices[ia * 3 + 1], worldVertices[ia * 3 + 2]);
            glm::vec3 b(worldVertices[ib * 3], worldVertices[ib * 3 + 1], worldVertices[ib * 3 + 2]);
            glm::vec3 c(worldVertices[ic * 3], worldVertices[ic * 3 + 1], worldVertices[ic * 3 + 2]);

            float ha = vertexHeights[ia];
            float hb = vertexHeights[ib];
            float hc = vertexHeights[ic];

            accumulateBuoyancy(a, b, c, comW, ha, hb, hc, obj->rho, obj->gravity, totalForce, totalTorque);
        }
        outForceTorque[0] = totalForce.x;
        outForceTorque[1] = totalForce.y;
        outForceTorque[2] = totalForce.z;
        outForceTorque[3] = totalTorque.x;
        outForceTorque[4] = totalTorque.y;
        outForceTorque[5] = totalTorque.z;
    }

    BUOYANCY_API void ComputeBuoyancy(
        int handle,
        const float* transformMatrix,
        const float* vertexHeights,
        int vertexHeightsCount,
        float* outForceTorque)
    {
        //Create matrix
        glm::mat4 m = glm::make_mat4(transformMatrix);
        //Make sure we have the object
        auto it = objects.find(handle);
        if (it == objects.end()) return;
        ObjectData* obj = it->second;
        
        //Transform to worldspace
        for (int i = 0; i < obj->vertexCount; i++)
        {
            glm::vec4 worldPos = m * glm::vec4(obj->vertices[i], 1.0f);
            obj->transformedVertices[i] = glm::vec3(worldPos);
        }

        // Transform com to world space
        glm::vec3 comWorld = glm::vec3(m * glm::vec4(obj->com, 1.0f));

        glm::vec3 totalForce(0.0f);
        glm::vec3 totalTorque(0.0f);

        for (size_t i = 0; i < obj->indexCount; i+=3)
        {
            int ia = obj->indices[i];
            int ib = obj->indices[i + 1];
            int ic = obj->indices[i + 2];

            glm::vec3 a = obj->transformedVertices[ia];
            glm::vec3 b = obj->transformedVertices[ib];
            glm::vec3 c = obj->transformedVertices[ic];
            float ha = vertexHeights[ia];
            float hb = vertexHeights[ib];
            float hc = vertexHeights[ic];

            int belowcount = checkUnderWater(a, b, c, ha, hb, hc);
            switch (belowcount)
            {
                case 0b000: continue;
                case 0b111: accumulateBuoyancy(a, b, c, comWorld, ha, hb, hc, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b100: oneBelowSplit(a, b, c, ha, hb, hc, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b010: oneBelowSplit(b, c, a, hb, hc, ha, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b001: oneBelowSplit(c, a, b, hc, ha, hb, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b011: oneAboveSplit(a, b, c, ha, hb, hc, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b101: oneAboveSplit(b, c, a, hb, hc, ha, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b110: oneAboveSplit(c, a, b, hc, ha, hb, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
            }
        }
        outForceTorque[0] = totalForce.x;
        outForceTorque[1] = totalForce.y;
        outForceTorque[2] = totalForce.z;
        outForceTorque[3] = totalTorque.x;
        outForceTorque[4] = totalTorque.y;
        outForceTorque[5] = totalTorque.z;
    }
    
    //Clear shit up
    BUOYANCY_API void DestroyBuoyancyInstance(int handle) {
        auto it = objects.find(handle);
        if (it != objects.end())
        {
            ObjectData* obj = it->second;
            delete[] obj->vertices;
            delete[] obj->indices;
            delete[] obj->transformedVertices;
            delete obj;
            objects.erase(it);
        }
    }

} // extern "C"

void oneAboveSplit(
    const glm::vec3& above,
    const glm::vec3& first,
    const glm::vec3& second,
    float hAbove,
    float hFirst,
    float hSecond,
    const glm::vec3& com,
    float rho, float gravity,
    glm::vec3& totalForce,
    glm::vec3& totalTorque)
{// Two triangles
    float dAbove = above.y - hAbove;
    float dFirst = first.y - hFirst;
    float dSecond = second.y - hSecond;

    float denom1 = (dFirst - dAbove);
    float denom2 = (dSecond - dAbove);
    if (fabsf(denom1) < 1e-6f || fabsf(denom2) < 1e-6f) return;


    //Got weird super high forces
    float step1 = glm::clamp(dFirst / denom1, 0.0f, 1.0f);
    float step2 = glm::clamp(dSecond / denom2, 0.0f, 1.0f);

    glm::vec3 p1 = glm::mix(first, above, step1);
    glm::vec3 p2 = glm::mix(second, above, step2);
    float hp1 = glm::mix(hFirst, hAbove, step1);
    float hp2 = glm::mix(hSecond, hAbove, step2);

    // Split up the quad into 2 triangles
    accumulateBuoyancy(first, second, p1, com, hFirst, hSecond, hp1, rho, gravity, totalForce, totalTorque);
    accumulateBuoyancy(second, p2, p1, com, hSecond, hp2, hp1, rho, gravity ,totalForce, totalTorque);
}

void oneBelowSplit(
    const glm::vec3& below,
    const glm::vec3& first,
    const glm::vec3& second,
    float hBelow,
    float hFirst,
    float hSecond,
    const glm::vec3& com,
    float rho, float gravity,
    glm::vec3& totalForce,
    glm::vec3& totalTorque)
{// End up with one triangle
    float dBelow = below.y - hBelow;
    float dFirst = first.y - hFirst;
    float dSecond = second.y - hSecond;
    //Got weird super high forces
    float step1 = glm::clamp(dFirst / (dFirst - dBelow), 0.0f, 1.0f);
    float step2 = glm::clamp(dSecond / (dSecond - dBelow), 0.0f, 1.0f);

    glm::vec3 p1 = glm::mix(first, below, step1);
    glm::vec3 p2 = glm::mix(second, below, step2);

    float hp1 = glm::mix(hFirst, hBelow, step1);
    float hp2 = glm::mix(hSecond, hBelow, step2);

    accumulateBuoyancy(below, p1, p2, com, hBelow, hp1, hp2, rho, gravity, totalForce, totalTorque);
}

void accumulateBuoyancy(
    const glm::vec3& a,
    const glm::vec3& b,
    const glm::vec3& c,
    const glm::vec3& com,
    float ha,
    float hb,
    float hc,
    float rho,
    float gravity,
    glm::vec3& totalForce,
    glm::vec3& totalTorque)
{
    float h = (ha+hb+hc)/3;

    // Force
    glm::vec3 areaVec = 0.5f * glm::cross(b - a, c - a);
    float sumY = a.y + b.y + c.y;

    float part1 = (rho * gravity / 3.0f);
    float scalar = part1 * (-3.0f * h + sumY);
    totalForce += scalar * areaVec;

    // Torque
    float S = glm::length(areaVec);
    if (S < 1e-12f) return;  // degenerate triangle, skip
    glm::vec3 n = areaVec / S;

    float sumX = a.x + b.x + c.x;
    float sumZ = a.z + b.z + c.z;
    // NOTE: Hirae et al. 2025 Eqs. 7 and 9, but they are wrong
    float deltaXZ = sumY - 4.0f * h;
    float deltaY  = 2.0f * sumY - 4.0f * h;

    glm::vec3 A;
    A.x = sumX * deltaXZ + (a.x * a.y + b.x * b.y + c.x * c.y);
    A.y = sumY * deltaY  - 2.0f * (a.y * b.y + b.y * c.y + c.y * a.y);
    A.z = sumZ * deltaXZ + (a.z * a.y + b.z * b.y + c.z * c.y);

    glm::vec3 innerPart = ((12.0f * h - 4.0f * sumY) * com) + A;
    float firstPart = part1 * (S / 4.0f);
    totalTorque += firstPart * glm::cross(innerPart, n);
}


// 0b000 dry, 0b100 a underneath, ... , 0b111 fully submerged
int checkUnderWater(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float ha, float hb, float hc) {
    return (((a.y - ha) < -EPS) << 2) | (((b.y-hb) < -EPS) << 1) | ((c.y - hc) < -EPS);
}