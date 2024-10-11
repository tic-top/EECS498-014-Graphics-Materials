#include <cstdint>
#include <iostream>
#include <limits>
#include "image.hpp"
#include "loader.hpp"
#include "rasterizer.hpp"

bool IsInsideTriangle(uint32_t x, uint32_t y, const Triangle &trig)
{
    // Embed the point X into 3D space
    glm::vec3 X(x, y, 0.0f);

    // Convert triangle vertices to 3D vectors
    glm::vec3 A1(trig.pos[0].x, trig.pos[0].y, trig.pos[0].z);
    glm::vec3 A2(trig.pos[1].x, trig.pos[1].y, trig.pos[1].z);
    glm::vec3 A3(trig.pos[2].x, trig.pos[2].y, trig.pos[2].z);

    // Calculate the cross products
    glm::vec3 S1 = glm::cross(X - A1, A2 - A1); // (X - A1) × (A2 - A1)
    glm::vec3 S2 = glm::cross(X - A2, A3 - A2); // (X - A2) × (A3 - A2)
    glm::vec3 S3 = glm::cross(X - A3, A1 - A3); // (X - A3) × (A1 - A3)

    // Extract z-components of the cross products
    float z1 = S1.z;
    float z2 = S2.z;
    float z3 = S3.z;

    // Check if all z-components have the same sign
    bool hasSameSign = (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);

    return hasSameSign;
}

void Rasterizer::DrawPixel(uint32_t x, uint32_t y, Triangle trig, AntiAliasConfig config, uint32_t spp, Image &image, Color color)
{
    if (config == AntiAliasConfig::NONE) // if anti-aliasing is off
    {
        if (IsInsideTriangle(x, y, trig))
        {
            image.Set(x, y, color);
        }
    }
    else if (config == AntiAliasConfig::SSAA) // if anti-aliasing is on
    {
        // spp (samples per pixel) defines how many sub-pixel samples we are going to check
        uint32_t count = 0; // count how many sample points are inside the triangle

        // Loop over each sample point in the pixel
        for (uint32_t i = 0; i < spp; ++i)
        {
            // Generate sample points within the pixel
            // For simplicity, use a regular grid sampling based on spp.
            // Example: For spp = 4, create a 2x2 grid of samples in the pixel.
            float sampleX = x + (i % int(sqrt(spp))) / float(sqrt(spp));
            float sampleY = y + (i / int(sqrt(spp))) / float(sqrt(spp));

            // Check if the sample point is inside the triangle
            if (IsInsideTriangle(sampleX, sampleY, trig))
            {
                ++count;
            }
        }

        // Calculate the final pixel color based on how many samples were inside the triangle
        float coverage = static_cast<float>(count) / spp; // percentage of samples inside the triangle
        Color finalColor = color * coverage;              // scale the color based on the coverage

        // Write the final color to the pixel
        image.Set(x, y, finalColor);
    }
    return;
}

void Rasterizer::AddModel(MeshTransform transform, glm::mat4 rotation)
{
    /* model.push_back( model transformation constructed from translation, rotation and scale );*/
    model.push_back(rotation * glm::translate(glm::mat4(1.0f), transform.translation) * glm::scale(glm::mat4(1.0f), transform.scale));
    return;
}

void Rasterizer::SetView()
{
    const Camera &camera = this->loader.GetCamera();
    glm::vec3 cameraPos = camera.pos;       // Camera position (eye)
    glm::vec3 cameraLookAt = camera.lookAt; // Target point (center)
    glm::vec3 cameraUp = camera.up;         // Up vector

    // Compute the forward (z-axis), right (x-axis), and up (y-axis) vectors
    glm::vec3 zaxis = glm::normalize(cameraLookAt - cameraPos);    // The "forward" vector
    glm::vec3 xaxis = glm::normalize(glm::cross(zaxis, cameraUp)); // The "right" vector
    glm::vec3 yaxis = glm::cross(xaxis, zaxis);                    // The "up" vector

    // Create the view matrix
    glm::mat4 view(1.0f);

    // Rotation part (note the positions due to column-major order)
    view[0][0] = xaxis.x;
    view[1][0] = xaxis.y;
    view[2][0] = xaxis.z;
    view[0][1] = yaxis.x;
    view[1][1] = yaxis.y;
    view[2][1] = yaxis.z;
    view[0][2] = -zaxis.x; // Negate to look down the negative z-axis
    view[1][2] = -zaxis.y;
    view[2][2] = -zaxis.z;

    // Translation part
    view[3][0] = -glm::dot(xaxis, cameraPos);
    view[3][1] = -glm::dot(yaxis, cameraPos);
    view[3][2] = glm::dot(zaxis, cameraPos); // Positive due to negation above

    // Assign the computed view matrix
    this->view = view;
    return;
}

void Rasterizer::SetProjection()
{
    const Camera &camera = this->loader.GetCamera();

    float nearClip = camera.nearClip;
    float farClip = camera.farClip;

    float width = camera.width;
    float height = camera.height;

    // Calculate the frustum boundaries
    float l = -width / 2.0f;  // Left
    float r = width / 2.0f;   // Right
    float b = -height / 2.0f; // Bottom
    float t = height / 2.0f;  // Top

    // Note: We keep n and f positive and adjust the formulas accordingly
    float n = nearClip;
    float f = farClip;

    glm::mat4 M_orth1 = glm::mat4(1.0f);
    M_orth1[0][0] = 2.0f / (r - l);
    M_orth1[1][1] = 2.0f / (t - b);
    M_orth1[2][2] = 2.0f / (n - f);

    glm::mat4 M_orth2 = glm::mat4(1.0f);
    M_orth2[3][0] = -(r + l) / 2;
    M_orth2[3][1] = -(t + b) / 2;
    M_orth2[3][2] = -(f + n) / 2;

    glm::mat4 M_orth = M_orth1 * M_orth2;

    glm::mat4 M_persp = glm::mat4(0.0f);
    M_persp[0][0] = n;
    M_persp[1][1] = n;
    M_persp[2][2] = n + f;
    M_persp[2][3] = -1.0f;
    M_persp[3][2] = -n * f;

    this->projection = M_orth * M_persp;
    return;

    // Compute the elements of the projection matrix with corrected signs
    float A = (2.0f * n) / (r - l);
    float B = (2.0f * n) / (t - b);
    float C = (r + l) / (r - l);
    float D = (t + b) / (t - b);
    float E = -(f + n) / (f - n);        // Corrected sign
    float F = -(2.0f * f * n) / (f - n); // Corrected sign

    // Initialize the projection matrix to zero
    glm::mat4 projection(0.0f);

    // Set the projection matrix elements
    projection[0][0] = A;     // m00
    projection[0][2] = C;     // m02
    projection[1][1] = B;     // m11
    projection[1][2] = D;     // m12
    projection[2][2] = E;     // m22
    projection[2][3] = -1.0f; // m23
    projection[3][2] = F;     // m32
    // Assign the computed projection matrix
    this->projection = projection;
    return;
}

void Rasterizer::SetScreenSpace()
{
    float width = this->loader.GetWidth();
    float height = this->loader.GetHeight();

    // Create the screenspace transformation matrix
    glm::mat4 screenspace(1.0f);

    // Scale NDC coordinates to [0, width] and [0, height]
    screenspace[0][0] = width / 2.0f;  // Scale X by width/2
    screenspace[1][1] = height / 2.0f; // Scale Y by height/2

    // Translate NDC coordinates from [-1,1] to [0, width] and [0, height]
    screenspace[3][0] = width / 2.0f;  // Translate X by width/2
    screenspace[3][1] = height / 2.0f; // Translate Y by height/2

    // Assign the computed screenspace matrix
    this->screenspace =  screenspace;
    this->screenspace = glm::mat4(1.0f);
    return;
}

glm::vec3 Rasterizer::BarycentricCoordinate(glm::vec2 pos, Triangle trig)
{

    glm::vec2 A = glm::vec2(trig.pos[0].x, trig.pos[0].y);
    glm::vec2 B = glm::vec2(trig.pos[1].x, trig.pos[1].y);
    glm::vec2 C = glm::vec2(trig.pos[2].x, trig.pos[2].y);
    glm::vec2 P = pos;

    // Compute the vectors from point A to B, A to C, and A to P
    glm::vec2 v0 = B - A;
    glm::vec2 v1 = C - A;
    glm::vec2 v2 = P - A;

    // Compute dot products
    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    // Compute the denominator of the barycentric coordinates
    float denom = d00 * d11 - d01 * d01;

    // Check for degenerate triangle (denominator should not be zero)
    if (denom == 0.0f)
    {
        // Handle degenerate case appropriately (returning zero or any default value)
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    // Compute barycentric coordinates
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    // check u, v, w times A,B,C = P
    // glm::vec2 P_check = u * A + v * B + w * C;

    return glm::vec3(u, v, w);
}

float Rasterizer::zBufferDefault = 1.0f; // Far plane in NDC

// TODO
void Rasterizer::UpdateDepthAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, ImageGrey &ZBuffer)
{
    // Compute the pixel center position
    glm::vec2 P(x + 0.5f, y + 0.5f); // Adding 0.5 to sample at the pixel center

    // Compute barycentric coordinates with respect to the transformed triangle
    glm::vec3 barycentricCoords = BarycentricCoordinate(P, transformed);

    float u = barycentricCoords.x;
    float v = barycentricCoords.y;
    float w = barycentricCoords.z;

    // Check if the point is inside the triangle
    if (u < 0.0f || v < 0.0f || w < 0.0f)
    {
        // The pixel is outside the triangle; no need to update the depth buffer
        return;
    }

    // Interpolate the depth value at the pixel using barycentric coordinates
    float depth = u * transformed.pos[0].z + v * transformed.pos[1].z + w * transformed.pos[2].z;

    // Ensure the depth value is between -1 and 1
    depth = glm::clamp(depth, -1.0f, 1.0f);

    // Perform depth testing
    std::optional<float> optCurrentDepth = ZBuffer.Get(x, y);

    float currentDepth = optCurrentDepth.has_value() ? optCurrentDepth.value() : zBufferDefault;

    if (depth < currentDepth)
    {
        // The new depth is closer; update the depth buffer
        ZBuffer.Set(x, y, depth);
    }

    // If the new depth is not closer, do nothing
}

void Rasterizer::ShadeAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, Image &image)
{                                    // Compute the pixel center position in screen space
    glm::vec2 P(x + 0.5f, y + 0.5f); // Sampling at the pixel center

    // Compute barycentric coordinates with respect to the transformed triangle
    glm::vec3 barycentricCoords = BarycentricCoordinate(P, transformed);

    float u = barycentricCoords.x;
    float v = barycentricCoords.y;
    float w = barycentricCoords.z;

    // Check if the point is inside the triangle
    if (u < 0.0f || v < 0.0f || w < 0.0f)
    {
        // The pixel is outside the triangle; no need to shade
        return;
    }

    // Interpolate the depth value at the pixel using barycentric coordinates
    float depth = u * transformed.pos[0].z + v * transformed.pos[1].z + w * transformed.pos[2].z;

    // Clamp depth to [-1, 1] to ensure it stays within NDC range
    depth = glm::clamp(depth, -1.0f, 1.0f);

    // Retrieve the depth value from ZBuffer at this pixel
    std::optional<float> optCurrentDepth = ZBuffer.Get(x, y);

    if (!optCurrentDepth.has_value())
    {
        // No depth value at this pixel, cannot shade
        return;
    }

    float currentDepth = optCurrentDepth.value();

    // Check if depth is exactly the value in ZBuffer
    const float epsilon = 1e-5f; // Small value to account for floating-point precision

    if (std::abs(depth - currentDepth) < epsilon)
    {
        // Compute the position in world space using barycentric coordinates on the original triangle
        glm::vec4 worldPos = u * original.pos[0] + v * original.pos[1] + w * original.pos[2];

        // Homogenize the world position (if w != 1)
        if (worldPos.w != 0.0f && worldPos.w != 1.0f)
        {
            worldPos /= worldPos.w;
        }

        // Compute the normal at the pixel
        glm::vec4 normal = u * original.normal[0] + v * original.normal[1] + w * original.normal[2];

        // Normalize the normal vector
        normal = glm::normalize(normal);

        // Retrieve ambient color from loader
        Color ambientColorObj = loader.GetAmbientColor();
        glm::vec3 ambientColor = glm::vec3(ambientColorObj.r, ambientColorObj.g, ambientColorObj.b) / 255.0f;

        // Retrieve specular exponent (shininess) from loader
        float shininess = loader.GetSpecularExponent();

        // Retrieve lights from loader
        const std::vector<Light> &lights = loader.GetLights();

        // Initialize final color components
        Color finalColor(0.0f, 0.0f, 0.0f, 255.0f);

        // For each light, compute its contribution
        for (const Light &light : lights)
        {
            // Light properties
            glm::vec3 lightPos = light.pos;    // Assuming Light has a position member
            Color lightColor = light.color;    // Assuming Light has a color member
            float intensity = light.intensity; // Assuming Light has an intensity member

            // Compute light direction
            glm::vec3 lightDir = glm::normalize(lightPos - glm::vec3(worldPos));

            // Compute view direction (assuming camera is at eye position)
            const Camera &camera = loader.GetCamera();
            glm::vec3 eyePos = camera.pos; // Assuming Camera has a position member
            glm::vec3 viewDir = glm::normalize(eyePos - glm::vec3(worldPos));
            float r = glm::length(lightPos - glm::vec3(worldPos));

            // Compute ambient component
            Color ambient = loader.GetAmbientColor();

            // Compute diffuse component
            float diff = std::max(glm::dot(glm::vec3(normal), lightDir), 0.0f);
            Color diffuse = diff * intensity / (r * r) * lightColor; // Assuming lightColor is in [0,1]

            // Compute specular component
            glm::vec3 halfDir = (lightDir + viewDir) * 0.5f;
            float spec = std::pow(std::max(glm::dot(glm::vec3(normal), halfDir), 0.0f), shininess);
            Color specular = spec * intensity / (r * r) * lightColor; // Assuming lightColor is in [0,1]

            // Combine components
            Color color = ambient + diffuse + specular;

            // Accumulate the color contributions from all lights
            finalColor = finalColor + color;
        }
        // Write the color to the image at (x, y)
        image.Set(x, y, finalColor);
    }

    // If depth does not match, do nothing
}
