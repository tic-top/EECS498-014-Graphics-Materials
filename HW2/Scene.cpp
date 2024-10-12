#include "Scene.h"
#include "Config.h"
#include <iostream>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

tinyobj::ObjReader Scene::reader {};

void Scene::addObjects(std::string_view modelPath, std::string_view searchPath) {
    tinyobj::ObjReaderConfig config;
    config.mtl_search_path = searchPath;
    if (!reader.ParseFromFile(std::string(modelPath), config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
            std::filesystem::path relative(modelPath);
            std::cerr << "Reading file " << std::filesystem::absolute(relative) << " error. File may be malformed or not exist.\n";
        }
        exit(1);
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        auto* object = new Object();
        object->name = shapes[s].name;
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            std::vector<Vec3> positions;
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
                tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
                tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

                positions.push_back({vx, vy, vz});
            } // per-vertex
            index_offset += fv;
            Mesh mesh {positions[0], positions[1], positions[2]};
            object->area += mesh.area;
            object->meshes.push_back(std::move(mesh));
        } // per-face
        object->constructBoundingBox();
        // we assume each object uses only a single material for all meshes
        auto materialId = shapes[s].mesh.material_ids[0];
        auto& material = materials[materialId];
        object->kd = Vec3 {
            material.diffuse[0],
            material.diffuse[1],
            material.diffuse[2],
        };
        if (material.emission[0] + 
            material.emission[1] + 
            material.emission[2] > 0) { // is light
            object->ke = Vec3 {
                material.emission[0], 
                material.emission[1],
                material.emission[2]
            };
            object->hasEmission = true;
            lights.push_back(object);
            lightArea += object->area;
        }
        objects.push_back(object);
    } // per-shape
}

void Scene::constructBVH() {
    assert (!objects.empty());
    bvh.root = BVH::build(objects);
}

Intersection Scene::getIntersection(const Ray &ray) {
    assert (bvh.root);
    return bvh.root->intersect(ray);
}

Intersection Scene::sampleLight() const {
    assert (lights.size() == 1 && "Currently only support a single light object");
    assert (lightArea > 0.0f);
    Intersection inter;
    return lights[0]->sample();
}

// direct
// Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
//     if constexpr (DEBUG) {
//         assert(ray.isNormalized());
//     }

//     // Use the getIntersection function to find the closest intersection
//     Intersection intersection = getIntersection(ray);
//     if (!intersection.happened) {
//         // If no intersection, return background color (black in this case)
//         return Vec3(0.0f, 0.0f, 0.0f);
//     }

//     // Direct lighting computation starts here
//     Vec3 Lo(0.0f, 0.0f, 0.0f);

//     // Get emitted radiance from the intersected object (if emissive)
//     Vec3 Le = intersection.getEmission();
    
//     if (!discardEmission && Le.getLength() > 0.0f) {
//         // Add emission directly if it's not discarded
//         Lo += Le;
//     }

//     // Sample a random direction on the hemisphere oriented around the normal
//     Vec3 normal = intersection.getNormal();
//     Vec3 wi = Random::randomHemisphereDirection(normal);

//     // Trace a ray in the sampled direction
//     Ray shadowRay(intersection.pos, wi);
//     Intersection secondIntersection = getIntersection(shadowRay);

//     if (secondIntersection.happened) {
//         // Incoming radiance from the light source or another object
//         Vec3 Li = secondIntersection.getEmission(); // Assuming the emission of intersected object is Li
        
//         // Calculate BRDF at the intersection point
//         Vec3 brdf = intersection.calcBRDF(-shadowRay.dir, -ray.dir); // ωi and ωo are reversed because we trace backward

//         // Calculate the cosine term (n · ωi)
//         float cosineTerm = std::max(0.0f, normal.dot(wi));

//         // Assume uniform sampling over the hemisphere for pdf
//         float pdf = 1.0f / (2.0f * PI);

//         // Apply the direct illumination equation
//         Lo += (Li * brdf * cosineTerm) / pdf;
//     }

//     // Return the outgoing radiance
//     return Lo;
// }

// global
// Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
//     if constexpr (DEBUG) {
//         assert(ray.isNormalized());
//     }

//     // Base case: stop recursion when no bounces are left
//     if (bouncesLeft <= 0) {
//         return Vec3(0.0f, 0.0f, 0.0f);  // Return zero radiance for no more bounces
//     }

//     // Use the getIntersection function to find the closest intersection
//     Intersection intersection = getIntersection(ray);
//     if (!intersection.happened) {
//         // If no intersection, return background color (black in this case)
//         return Vec3(0.0f, 0.0f, 0.0f);
//     }

//     // Direct lighting computation starts here
//     Vec3 Lo(0.0f, 0.0f, 0.0f);

//     // Get emitted radiance from the intersected object (if emissive)
//     Vec3 Le = intersection.getEmission();

//     if (!discardEmission) {
//         // Add emission directly if it's not discarded
//         Lo += Le;
//     }

//     // Sample a random direction on the hemisphere oriented around the normal
//     Vec3 normal = intersection.getNormal();
//     Vec3 wi = Random::randomHemisphereDirection(normal);

//     // Trace a ray in the sampled direction (this will handle indirect lighting)
//     Ray nextRay(intersection.pos, wi);

//     // Recursively trace the next ray for indirect lighting contribution
//     Vec3 indirectRadiance = trace(nextRay, bouncesLeft - 1, discardEmission);

//     // Calculate BRDF at the intersection point
//     Vec3 brdf = intersection.calcBRDF(-nextRay.dir, -ray.dir); // ωi and ωo are reversed because we trace backward

//     // Calculate the cosine term (n · ωi)
//     float cosineTerm = std::max(0.0f, normal.dot(wi));

//     // Assume uniform sampling over the hemisphere for pdf
//     float pdf = 1.0f / (2.0f * PI);

//     // Add indirect illumination to the outgoing radiance
//     Lo += (indirectRadiance * brdf * cosineTerm) / pdf;

//     // Return the outgoing radiance
//     return Lo;
// }

// acc
Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
    if constexpr (DEBUG) {
        assert(ray.isNormalized());
    }

    // Base case: stop recursion when no bounces are left
    if (bouncesLeft <= 0) {
        return Vec3(0.0f, 0.0f, 0.0f);  // Return zero radiance if no more bounces
    }

    // Find intersection of the ray with the scene
    Intersection intersection = getIntersection(ray);
    if (!intersection.happened) {
        // If no intersection, return background color (black in this case)
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    // Initialize the outgoing radiance
    Vec3 Lo(0.0f, 0.0f, 0.0f);

    // Get emitted radiance from the intersected object (if emissive)
    Vec3 Le = intersection.getEmission();
    if (!discardEmission) {
        // Add emission directly if not discarded
        Lo += Le;
    }

    // Handle direct lighting by sampling the light source
    Intersection lightSample = sampleLight();
    Vec3 lightDir = lightSample.pos - intersection.pos;
    float distanceToLight = lightDir.getLength();
    lightDir.normalize();

    // Create a shadow ray towards the light source
    Ray rayToLight(intersection.pos, lightDir);

    // Check if the light is visible (not blocked by any objects)
    Intersection shadowIntersection = getIntersection(rayToLight);
    if (shadowIntersection.happened && shadowIntersection.getEmission().getLength()!=0) {
        // If the light is not blocked, calculate direct radiance contribution
        Vec3 lightRadiance = lightSample.getEmission();
        Vec3 brdf = intersection.calcBRDF(-lightDir, -ray.dir);  // ωi and ωo are reversed
        float cosTheta = std::max(0.0f, intersection.getNormal().dot(lightDir));
        float cosThetaLight = std::max(0.0f, -lightSample.getNormal().dot(lightDir));
        float pdfLightSample = 1.0f / lightArea;  // Access the precomputed light area

        // Add direct radiance contribution to the total outgoing radiance
        Lo += (lightRadiance * brdf * cosTheta * cosThetaLight) /
              (distanceToLight * distanceToLight * pdfLightSample);
    }

    // Handle indirect lighting using importance sampling (cosine-weighted hemisphere)
    Vec3 normal = intersection.getNormal();
    Vec3 wi = Random::cosWeightedHemisphere(normal);  // Sample a direction

    // Probability density function for cosine-weighted sampling
    float pdf = normal.dot(wi) / PI;

    // Trace a ray in the sampled direction (indirect radiance)
    Ray nextRay(intersection.pos, wi);
    Vec3 indirectRadiance = trace(nextRay, bouncesLeft - 1, true);  // Recursively trace

    // Calculate the BRDF and cosine term
    Vec3 brdf = intersection.calcBRDF(-wi, -ray.dir);  // ωi and ωo are reversed
    float cosineTerm = std::max(0.0f, normal.dot(wi));

    // Add indirect radiance contribution to the total outgoing radiance
    Lo += (indirectRadiance * brdf * cosineTerm) / pdf;

    // Return the total outgoing radiance (direct + indirect)
    return Lo;
}

Scene::~Scene() {
    for (auto obj : objects) {
        delete obj;
    }
}
