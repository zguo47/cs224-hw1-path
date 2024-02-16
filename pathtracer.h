#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"

struct Settings {
    int samplesPerPixel;
    bool directLightingOnly; // if true, ignore indirect lighting
    int numDirectLightingSamples; // number of shadow rays to trace from each intersection point
    float pathContinuationProb; // probability of spawning a new secondary ray == (1-pathTerminationProb)
};

class PathTracer
{
public:
    PathTracer(int width, int height);

    void traceScene(QRgb *imageData, const Scene &scene);
    Settings settings;

private:
    int m_width, m_height;

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix, int sampleIndex);
    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, bool count_emitted, int sampleIndex);
    Eigen::Vector3f BRDF(const tinyobj::material_t& mat, Eigen::Vector3f inputRay, IntersectionInfo i, Eigen::Vector3f newRayDir);
    std::pair<Eigen::Vector3f, float> sampleNextDir(int sampleIndex, const tinyobj::material_t& mat, Eigen::Vector3f inputRay, IntersectionInfo i);
    Eigen::Vector3f directLighting(const Ray& r, const Scene& scene, IntersectionInfo i, int sampleIndex);
    float triangleArea(const Eigen::Vector3<Eigen::Vector3f>& vertices);
    float vanDerCorput(int n, const int &base = 2);
    Eigen::Vector3f samplePointOnTriangle(const Eigen::Vector3<Eigen::Vector3f>& vertices, int sampleIndex);
    Eigen::Vector3f sampleDisk(float radius);

    bool isPrime(int n);
    int nextPrime(int n);
};

#endif // PATHTRACER_H
