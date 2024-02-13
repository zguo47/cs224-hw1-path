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

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, bool count_emitted);
    Eigen::Vector3f BRDF(const tinyobj::material_t&, Eigen::Vector3f inputRay, IntersectionInfo i, Eigen::Vector3f newRayDir);
    std::pair<Eigen::Vector3f, float> sampleNextDir();
    Eigen::Vector3f directLighting(const Ray& r, const Scene& scene, IntersectionInfo i);
    float triangleArea(const Eigen::Vector3<Eigen::Vector3f>& vertices);
    Eigen::Vector3f samplePointOnTriangle(const Eigen::Vector3<Eigen::Vector3f>& vertices);
    void fresnel(const Eigen::Vector3f I, const Eigen::Vector3f normal, const float ior, float kr);
};

#endif // PATHTRACER_H
