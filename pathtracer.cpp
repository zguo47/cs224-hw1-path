#include "pathtracer.h"

#include <iostream>

#include <util/CS123SceneData.h>

#include <Eigen/Dense>

#include <util/CS123Common.h>

#include <math.h>

using namespace Eigen;

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        #pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            for (int i = 0; i < settings.samplesPerPixel; i++){
                intensityValues[offset] += tracePixel(x, y, scene, invViewMat);
            }
            intensityValues[offset] /= settings.samplesPerPixel;
        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f p(0, 0, 0);
    Vector3f d((2.f * x / m_width) - 1, 1 - (2.f * y / m_height), -1);
    d.normalize();

    Ray r(p, d);
    r = r.transform(invViewMatrix);
    return traceRay(r, scene, true);
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, bool count_emitted)
{
    IntersectionInfo i;
    Ray ray(r);

    if(scene.getIntersection(ray, &i)) {

          //** Example code for accessing materials provided by a .mtl file **
       const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
       const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
       const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
       const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name
       Vector3f kd;
       kd = Vector3f(d[0], d[1], d[2]);

       float epsilon = 0.001;
       Vector3f newRayDir;
       Vector3f brdf;
       float pdf;
       std::tie(newRayDir, pdf) = sampleNextDir();

       auto CTM = Quaternionf::FromTwoVectors(Vector3f(0.0f, 1.0f, 0.0f), i.object->getNormal(i));
       newRayDir = CTM * newRayDir;

       Vector3f color = directLighting(r, scene, i);

       if((static_cast<float>(rand()) / RAND_MAX) < settings.pathContinuationProb && !settings.directLightingOnly) {
           Ray newRay(i.hit + newRayDir * epsilon, newRayDir);
           brdf = BRDF(mat, r.d, i, newRayDir);
           //mirror
           if (mat.diffuse[0] < 0.02 && mat.diffuse[1] < 0.02 && mat.diffuse[2] < 0.02 &&
               mat.specular[0] > 0.9 && mat.specular[1] > 0.9 && mat.specular[2] > 0.9){

               Vector3f ks(mat.specular[0], mat.specular[1], mat.specular[2]);
               Vector3f viewDir = - r.d.normalized();
               Vector3f reflectedRay = 2 * viewDir.dot(i.object->getNormal(i)) * i.object->getNormal(i) - viewDir;
               reflectedRay.normalize();
               Ray newReflectedRay(i.hit + reflectedRay * epsilon, reflectedRay);
               color += traceRay(newReflectedRay, scene, true).cwiseProduct(ks) / settings.pathContinuationProb;
           //refraction
           }else if (mat.diffuse[0] < 0.02 && mat.diffuse[1] < 0.02 && mat.diffuse[2] < 0.02 ){
               Vector3f refnormal = i.object->getNormal(i);
               float NdotI = refnormal.dot(r.d);
               float etai = 1.0f;
               float etat = mat.ior;
               if (NdotI < 0){
                   NdotI = -NdotI;
               }else{
                   refnormal = -refnormal;
                   std::swap(etai, etat);
               }
               float eta = etai / etat;
               float k = 1 - eta * eta * (1 - NdotI * NdotI);
               float cosTheta_t = sqrt(k);
               if (k < 0){
                   Vector3f ks(mat.specular[0], mat.specular[1], mat.specular[2]);
                   Vector3f viewDir = - r.d.normalized();
                   Vector3f reflectedRay = 2 * viewDir.dot(i.object->getNormal(i)) * i.object->getNormal(i) - viewDir;
                   reflectedRay.normalize();
                   Ray newReflectedRay(i.hit + reflectedRay * epsilon, reflectedRay);
                   color += traceRay(newReflectedRay, scene, true).cwiseProduct(ks) / settings.pathContinuationProb;
               }else{
                   float R0 = ((etai - etat) / (etai + etat)) * ((etai - etat) / (etai + etat));
                   float R_thetai = R0 + (1 - R0) * (1 - NdotI) * (1 - NdotI) * (1 - NdotI) * (1 - NdotI) * (1 - NdotI);
                   Vector3f refractedRay = eta * r.d + (eta * NdotI - cosTheta_t)  * refnormal;
                   refractedRay.normalize();
                   Ray newRefractedRay(i.hit + refractedRay * epsilon, refractedRay);

                   Vector3f ks(mat.specular[0], mat.specular[1], mat.specular[2]);
                   Vector3f viewDir = - r.d.normalized();
                   Vector3f reflectedRay = 2 * viewDir.dot(i.object->getNormal(i)) * i.object->getNormal(i) - viewDir;
                   reflectedRay.normalize();
                   Ray newReflectedRay(i.hit + reflectedRay * epsilon, reflectedRay);
                   if (((double) rand() / (RAND_MAX)) > R_thetai){
                        color += traceRay(newRefractedRay, scene, true) / settings.pathContinuationProb;
                   }else{
                       color += traceRay(newReflectedRay, scene, true).cwiseProduct(ks) / settings.pathContinuationProb;
                   }

               }
           }
           else{
               color += traceRay(newRay, scene, false).cwiseProduct(brdf) * (newRayDir.dot(i.object->getNormal(i))) / (pdf * settings.pathContinuationProb);
           }
       }
       if (count_emitted){
           color += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);
       }

        return color;
    } else {
        return Vector3f(0, 0, 0);
    }
}

Vector3f PathTracer::BRDF(const tinyobj::material_t& mat, Vector3f inputRay, IntersectionInfo i, Vector3f newRayDir) {
    Vector3f output;
    Vector3f color(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
    //glossy
    if (mat.diffuse[0] > 0.4 && mat.diffuse[1] > 0.4 && mat.diffuse[2] > 0.4
               && mat.specular[0] > 0.75 && mat.specular[1] > 0.75 && mat.specular[2] > 0.75){
        int n = mat.shininess;
        Vector3f ks(mat.specular[0], mat.specular[1], mat.specular[2]);
        Vector3f viewDir = - inputRay.normalized();
        Vector3f reflectedRay = 2 * viewDir.dot(i.object->getNormal(i)) * i.object->getNormal(i) - viewDir;
        reflectedRay.normalize();
        newRayDir.normalize();
        output = ks * (n + 2)/(2 * M_PI) * pow((reflectedRay.dot(newRayDir)), n);
    //diffuse
    }else{
        output = color / M_PI;
    }

    return output;
}

Vector3f PathTracer::directLighting(const Ray& r, const Scene& scene, IntersectionInfo i){
    Vector3f directLight(0.0f, 0.0f, 0.0f); // Initialize direct light contribution to black
    float area = 0.0f;

    const std::vector<Triangle*>& emissives = scene.getEmissives();
    for (Triangle* emissiveTriangle : emissives) {
        // Calculate direction from point of intersection to light source
        Vector3f samplePoint = samplePointOnTriangle(emissiveTriangle->getVertices());
        area = triangleArea(emissiveTriangle->getVertices());

        Vector3f lightDir = samplePoint - i.hit;
        float distanceSquared = lightDir.squaredNorm();
        lightDir.normalize();

        // Check visibility (shadow ray)
        Ray shadowRay(i.hit + i.object->getNormal(i) * 0.001f, lightDir);
        IntersectionInfo shadowInfo;
        if (scene.getIntersection(shadowRay, &shadowInfo) && shadowInfo.t < sqrt(distanceSquared) - 0.01f * sqrt(distanceSquared)) {
            continue;
        }

        // Calculate the dot product between the light direction and the normal at the intersection
        float NdotL = i.object->getNormal(i).dot(lightDir);
        float NdotLPrime = emissiveTriangle->getNormal(samplePoint).dot(-lightDir);

        Vector3f emittedLight = Vector3f(emissiveTriangle->getMaterial().emission[0],
                                         emissiveTriangle->getMaterial().emission[1],
                                         emissiveTriangle->getMaterial().emission[2]);

        if (NdotL + 0.001f > 0) { // Add light contribution if the surface faces the light
            const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
            const tinyobj::material_t& mat = t->getMaterial();
            if (mat.diffuse[0] > 0.02 && mat.diffuse[1] > 0.02 && mat.diffuse[2] > 0.02){
                directLight += emittedLight.cwiseProduct(BRDF(mat, r.d, i, lightDir)) * NdotL * NdotLPrime * area / (distanceSquared);
            }
        }
    }

    return directLight;
}

Vector3f PathTracer::samplePointOnTriangle(const Vector3<Vector3f>& vertices) {
    // Generate two random numbers
    float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float r2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    // Barycentric coordinates for uniform triangle sampling
    float sqrtR1 = sqrt(r1);
    float u = 1.0f - sqrtR1;
    float v = r2 * sqrtR1;

    // Calculate the sampled point on the triangle
    return vertices[0] * u + vertices[1] * v + vertices[2] * (1.0f - u - v);
}

float PathTracer::triangleArea(const Eigen::Vector3<Eigen::Vector3f>& vertices) {
    Eigen::Vector3f v1 = vertices[0];
    Eigen::Vector3f v2 = vertices[1];
    Eigen::Vector3f v3 = vertices[2];

    Eigen::Vector3f edge1 = v2 - v1;
    Eigen::Vector3f edge2 = v3 - v1;

    Eigen::Vector3f crossProduct = edge1.cross(edge2);
    float area = 0.5f * crossProduct.norm();

    return area;
}

std::pair<Vector3f, float> PathTracer::sampleNextDir() {
    float r1 = static_cast<float>(rand()) / RAND_MAX;
    float r2 = static_cast<float>(rand()) / RAND_MAX;

    // Convert uniform random numbers to spherical coordinates
    float phi = 2.0f * M_PI * r1;
    float theta = acos(1.0f - r2);

    float x = sin(theta) * cos(phi);
    float z = sin(theta) * sin(phi);
    float y = cos(theta);

    Vector3f sampleDir(x, y, z);

    float pdf = 1.0f / (2.0f * M_PI);

    return std::make_pair(sampleDir, pdf);
}


void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    const float gamma = 2.2f;
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f newcolor = intensityValues[offset].cwiseQuotient(Vector3f(1.0f, 1.0f, 1.0f) + intensityValues[offset]);
            Vector3f gammaCorrectedColor;
            gammaCorrectedColor[0] = std::pow(newcolor[0], 1.0f / gamma);
            gammaCorrectedColor[1] = std::pow(newcolor[1], 1.0f / gamma);
            gammaCorrectedColor[2] = std::pow(newcolor[2], 1.0f / gamma);

            imageData[offset] = qRgb(
                std::min(255.0f, std::max(0.0f, gammaCorrectedColor[0] * 255)),
                std::min(255.0f, std::max(0.0f, gammaCorrectedColor[1] * 255)),
                std::min(255.0f, std::max(0.0f, gammaCorrectedColor[2] * 255))
                );
        }
    }
    // outputPFM("/Users/shania/cs2240/path-zguo47/student_outputs/final/cornell_box_full_lighting.pfm", 512, 512, intensityValues);

}
