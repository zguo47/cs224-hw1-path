#include "pathtracer.h"

#include <iostream>

#include <util/CS123SceneData.h>

#include <Eigen/Dense>

#include <util/CS123Common.h>

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
            for (int i = 0; i < 40; i++){
                intensityValues[offset] += tracePixel(x, y, scene, invViewMat);
            }
            intensityValues[offset] /= 40.0f;
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
    return traceRay(r, scene);
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene)
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
       if (d[0] < 0.02 && d[1] < 0.02 && d[2] < 0.02){
           kd = Vector3f(1, 1, 1);
       }else{
           kd = Vector3f(d[0], d[1], d[2]);
       }

       Vector3f color(mat.emission[0], mat.emission[1], mat.emission[2]);

       float epsilon = 0.001;
       Vector3f newRayDir;
       Vector3f brdf;
       float pdf;
       std::tie(newRayDir, pdf) = sampleNextDir();

       auto CTM = Quaternionf::FromTwoVectors(Vector3f(0.0f, 1.0f, 0.0f), i.object->getNormal(i));
       newRayDir = CTM * newRayDir;

       if((static_cast<float>(rand()) / RAND_MAX) < settings.pathContinuationProb) {
           Ray newRay(i.hit + newRayDir * epsilon, newRayDir);
           brdf = BRDF(kd);
           color += traceRay(newRay, scene).cwiseProduct(brdf) * (newRayDir.dot(i.object->getNormal(i))) / (pdf * settings.pathContinuationProb);
       }

        return color;
    } else {
        return Vector3f(0, 0, 0);
    }
}

Vector3f PathTracer::BRDF(Vector3f &color) {
    return color / M_PI;
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
    outputPFM("/Users/shania/cs2240/path-zguo47/student_outputs/milestone/cornell_box_milestone.pfm", 512, 512, intensityValues);

}
