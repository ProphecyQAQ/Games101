//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <mutex>

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f Scene::shade(const Ray &ray, Intersection intersection) const
{
    Object *hitObject = intersection.obj;
    Material *m = intersection.m;

    if (m->hasEmission()) return m->getEmission();

    Vector3f N = intersection.normal; // normal
    Vector3f hitPoint = intersection.coords;
    Vector3f wo = ray.direction;

    //Contribution from the light source
    Vector3f L_dir(0.f, 0.f, 0.f);
    float pdf_light;
    Intersection hitLig;
    sampleLight(hitLig, pdf_light);

    Ray dirRay(hitLig.coords, (hitPoint - hitLig.coords).normalized()); // light to obj

    if (intersect(dirRay).obj == hitObject) {
        Vector3f NN = hitLig.normal;
        Vector3f ws = (hitLig.coords - hitPoint).normalized();

        float cosTheta    = std::max(dotProduct(ws, N), 0.f);
        float cosThetaPri = std::max(dotProduct(-ws, NN), 0.f);
        float r2 = dotProduct(hitLig.coords - hitPoint, hitLig.coords - hitPoint);

        L_dir = hitLig.emit * m->eval(ws, -wo, N) * cosTheta * cosThetaPri / r2 / pdf_light;
    }

    // Contribution from other reflectors.
    
    Vector3f L_indir(0.f, 0.f, 0.f);
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = m->sample(ray.direction, N).normalized();
        Ray inDirRay(hitPoint, wi);
        Intersection nexObj = intersect(inDirRay);

        if (nexObj.happened == true && !nexObj.m->hasEmission()) {
            float pdf = m->pdf(wo, wi, N) + 0.0001f;

            L_indir += shade(inDirRay, nexObj) * m->eval(wi, -wo, N) * std::max(0.f, dotProduct(wi, N)) / pdf / RussianRoulette; 
        }
    }

    return L_dir + L_indir;
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    if (intersection.happened == false) return {}; 
    return shade(ray, intersection);

}
