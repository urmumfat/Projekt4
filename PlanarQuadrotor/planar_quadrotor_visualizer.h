#pragma once

#include <memory>

#include <SDL.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor* quadrotor_ptr;
    int screen_w;
    int screen_h;
    float rotor_radiusX = 15.0f;

public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr, int screen_w, int screen_h);
    void render(std::shared_ptr<SDL_Renderer>& gRenderer);
    void MouseClickfollow(int x, int y);
};