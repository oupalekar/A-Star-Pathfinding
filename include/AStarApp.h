//
// Created by Ojas Upalekar on 4/22/21.
//
#ifndef A_STAR_ASTARAPP_H
#define A_STAR_ASTARAPP_H
#pragma once

#include "cinder/gl/gl.h"
#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "Pathfinder.h"

namespace visualizer {
  using pathfinder::Pathfinder;
  using glm::vec2;
  class AStarApp : public ci::app::App{
    public:
      AStarApp();
      
      void draw() override;
      void update() override;
      
      const int kWindowSize = 800;
      const int kMargin = 10;
      vec2 top_left = vec2(kMargin,kMargin);
      int pixel_side_length;

    private:
      Pathfinder app = Pathfinder(0, 0);
  };
}

#endif //A_STAR_ASTARAPP_H
