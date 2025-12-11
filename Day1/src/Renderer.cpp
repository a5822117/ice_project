//
// Created by kango on 2023/04/03.
//

#include "Renderer.h"

#include <utility>
#include <iostream>

Renderer::Renderer(const std::vector<Body> &bodies, Camera camera, Color bgColor)
: bodies(bodies), camera(std::move(camera)), bgColor(std::move(bgColor)) {}

/**
 * \b シーン内に存在するBodyのうちレイにhitするものを探す
 * @param ray レイ
 * @param hit hitした物体の情報を格納するRayHit構造体
 * @return 何かしらのBodyにhitしたかどうかの真偽値
 */
bool Renderer::hitScene(const Ray &ray, RayHit &hit) const {
    /// hitするBodyのうち最小距離のものを探す
    hit.t = DBL_MAX;//
    hit.idx = -1;
    for(int i = 0; i < bodies.size();++ i) {
        RayHit _hit;
        if(bodies[i].hit(ray, _hit) && _hit.t < hit.t) {//一番手前に当たったものを返す
            hit.t = _hit.t;
            hit.idx = i;
            hit.point = _hit.point;
            hit.normal = _hit.normal;
        }
    }

    return hit.idx != -1;
}

Image Renderer::render() const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());
    /// フィルム上のピクセル全てに向けてレイを飛ばす
    for(int p_y = 0; p_y < image.height; ++p_y) {
        for(int p_x = 0; p_x < image.width; ++p_x) {
            const int p_idx = p_y * image.width + p_x;
            Color color;
            Ray ray; RayHit hit;
            camera.filmView(p_x, p_y, ray);
            color = hitScene(ray, hit) ? bodies[hit.idx].material.color : bgColor;
            image.pixels[p_idx] = color;
        }
    }

    return image;
}
