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

int checkUnderWater(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
void accumulateBuoyancy(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& com, float rho, float gravity, glm::vec3& totalForce, glm::vec3& totalTorque);
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

    BUOYANCY_API void ComputeBuoyancy(
        int handle,
        const float* transformMatrix,
        const float* waveParams,
        int algoChoice,
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

        // So now I have everything in world space, then we check for each 
        // triangle if they're y value is below water, right now 0 but later
        // replaced with function to get the water height at the point

        // A triangle is a tripplet of indexes, each index points to vertices
        // in the triangle
        // Move out to own function

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

            int belowcount = checkUnderWater(a, b, c);
            switch (belowcount)
            {
                case 0b000: continue;
                case 0b111: accumulateBuoyancy(a, b, c, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b100: oneBelowSplit(a, b, c, 0.0f, 0.0f, 0.0f, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b010: oneBelowSplit(b, c, a, 0.0f, 0.0f, 0.0f, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b001: oneBelowSplit(c, a, b, 0.0f, 0.0f, 0.0f, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b011: oneAboveSplit(a, b, c, 0.0f, 0.0f, 0.0f, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b101: oneAboveSplit(b, c, a, 0.0f, 0.0f, 0.0f, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
                case 0b110: oneAboveSplit(c, a, b, 0.0f, 0.0f, 0.0f, comWorld, obj->rho, obj->gravity, totalForce, totalTorque); break;
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

    float step1 = dFirst / (dFirst - dAbove);
    float step2 = dSecond / (dSecond - dAbove);
    glm::vec3 p1 = glm::mix(first, above, step1);
    glm::vec3 p2 = glm::mix(second, above, step2);
    // Split up the quad into 2 triangles
    accumulateBuoyancy(first, second, p1, com, rho, gravity, totalForce, totalTorque);
    accumulateBuoyancy(second, p2, p1, com, rho, gravity, totalForce, totalTorque);
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

    float step1 = dFirst / (dFirst - dBelow);
    float step2 = dSecond / (dSecond - dBelow);
    glm::vec3 p1 = glm::mix(first, below, step1);
    glm::vec3 p2 = glm::mix(second, below, step2);
    accumulateBuoyancy(below, p1, p2, com, rho, gravity, totalForce, totalTorque);
}

void accumulateBuoyancy(
    const glm::vec3& a,
    const glm::vec3& b,
    const glm::vec3& c,
    const glm::vec3& com,
    float rho, float gravity,
    glm::vec3& totalForce,
    glm::vec3& totalTorque)
{
    float h = 0.0f;  // flat water, later we get with function

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
    // NOTE: Hirae et al. 2025 Eqs. 7 and 9 print "{-4h + 2(y1+y2+y3)}" for A.x and A.z,
    // but deriving the integral from scratch gives "{-4h + (y1+y2+y3)}" for those two.
    // The "2*sumY" form is only correct for A.y, where the doubling is absorbed by the
    // -2*Σ(pairwise y products) term via the identity sumY^2 = Σy^2 + 2*Σ(pairs).
    // Using the paper's printed form for A.x/A.z gives a spurious sumX*sumY (resp.
    // sumZ*sumY) term per triangle that does not cancel over a closed mesh, producing
    // a constant torque bias. See verification with triangle (0,0,0),(1,0,0),(0,1,0).
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
int checkUnderWater(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    return ((a.y < -EPS) << 2) | ((b.y < -EPS) << 1) | (c.y < -EPS);
}