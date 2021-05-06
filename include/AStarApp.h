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
      void update() override;
      void updateBoard(const vec2& coord, size_t value);
      void setStartNode(const vec2& coord);
      void setEndNode(const vec2& coord);
      void CreatePath();
      void DrawConnections();
      
      const int kWindowSize = 800;
      const int kMargin = 10;
      const size_t num_of_nodes_side = 16;
      vec2 top_left = vec2(kMargin,kMargin);
      const double pixel_side_length = (kWindowSize - ((num_of_nodes_side + 1) * kMargin)) / num_of_nodes_side;
      double radius = pixel_side_length / 2;
      ci::Rectf double_button = ci::Rectf(vec2(820, 20), vec2(850, 50));
      
    private:
      Pathfinder app = Pathfinder(0, 0);
      vector<vector<size_t>> value_of_nodes_;

      void CheckVisited();

      void DrawStrings();
  };
}

#endif //A_STAR_ASTARAPP_H
