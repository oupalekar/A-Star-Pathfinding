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
#include <vector>

namespace visualizer {
  using std::vector;
  using pathfinder::Pathfinder;
  using glm::vec2;
  class AStarApp : public ci::app::App{
    public:
      AStarApp();
      
      void draw() override;
      void mouseDown(ci::app::MouseEvent event) override;
      void mouseDrag(ci::app::MouseEvent event) override;
      void updateBoard(const vec2& coord);
      void setStartNode(const vec2& coord);
      void setEndNode(const vec2& coord);
      
      const int kWindowSize = 800;
      const int kMargin = 10;
      vec2 top_left = vec2(kMargin,kMargin);
      int pixel_side_length;
      double brush_radius = .5;

    private:
      Pathfinder app = Pathfinder(0, 0);
      vector<vector<size_t>> value_of_nodes_;
  };
}

#endif //A_STAR_ASTARAPP_H
