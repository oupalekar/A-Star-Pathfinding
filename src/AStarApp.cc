//
// Created by Ojas Upalekar on 4/22/21.
//

#include "AStarApp.h"
#include "Pathfinder.h"

namespace visualizer {
  using glm::vec2;
  using pathfinder::Pathfinder;
  using std::cout;

  AStarApp::AStarApp() {
      ci::app::setWindowSize(kWindowSize, kWindowSize);
      app = Pathfinder(20, 20);
      app.setDiagonals(true);
      app.CreateNodes();
      value_of_nodes_ = vector<vector<size_t>>(app.getNumOfCols(), vector<size_t>(app.getNumOfRows(), 0));
      pixel_side_length = (kWindowSize - (2 * kMargin)) / app.getNumOfCols();
  }

  void AStarApp::draw() {
      for (size_t row = 0; row < app.getNumOfRows(); ++row) {
          for (size_t col = 0; col < app.getNumOfCols(); ++col) {
              if (value_of_nodes_[row][col] == 1) { //Checks if the shade of the pixel is not 0
                  ci::gl::color(ci::Color::gray(0.3F));
              } else if (value_of_nodes_[row][col] == 2) {
                  ci::gl::color(ci::Color("green"));
              } else if (value_of_nodes_[row][col] == 3) {
                  ci::gl::color(ci::Color("red"));
              } else if (value_of_nodes_[row][col] == 4) {
                  ci::gl::color(ci::Color("yellow"));
              } else {
                  ci::gl::color(ci::Color("white"));
              }
              vec2 pixel_top_left = top_left + vec2(col * pixel_side_length,
                                                    row * pixel_side_length);

              vec2 pixel_bottom_right =
                      pixel_top_left + vec2(pixel_side_length, pixel_side_length);
              ci::Rectf pixel_bounding_box(pixel_top_left, pixel_bottom_right);

              ci::gl::drawSolidRect(pixel_bounding_box);

              ci::gl::color(ci::Color("blue"));
              ci::gl::drawStrokedRect(pixel_bounding_box);
          }
      }
      if (app.getStartNode() != nullptr && app.getEndNode() != nullptr) {
          app.SolveAStar();
          RemovePath();
          Pathfinder::Node *temp = app.getEndNode();
          temp = temp->parent;
          while (temp->parent != nullptr) {
              value_of_nodes_[temp->y_][temp->x_] = 4;
              temp = temp->parent;
          }
      }
  }

  void AStarApp::mouseDown(ci::app::MouseEvent event) {
      if (event.isControlDown()) {
          setStartNode(event.getPos());
          //CreatePath();
      } else if (event.isShiftDown()) {
          setEndNode(event.getPos());
          //CreatePath();
      } else {
          if(event.isRight()) {
              updateBoard(event.getPos(), 0);
          }
          updateBoard(event.getPos(), 1);
      }
  }

  void AStarApp::mouseDrag(ci::app::MouseEvent event) {
      if(event.isRight()) {
          updateBoard(event.getPos(), 0);
      } else {
          updateBoard(event.getPos(), 1);
      }
  }

  void AStarApp::updateBoard(const vec2 &coord, size_t value) {
      double radius = brush_radius;
      vec2 brush_sketchpad_coords =
              (coord - top_left) / (float) pixel_side_length;
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              vec2 pixel_center = {col + 0.5, row + 0.5};
              if (glm::distance(brush_sketchpad_coords, pixel_center) <=
                  radius) {
                  value_of_nodes_[row][col] = value; //If the distance between the pixels and the
                  // brush is <= the radius then sets that pixel to a shade of 1
                  app.setObstacle(col, row, true);
              }
          }
      }
  }

  void AStarApp::setStartNode(const vec2 &coord) {
      vec2 brush_sketchpad_coords =
              (coord - top_left) / (float) pixel_side_length;
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              vec2 pixel_center = {col + 0.5, row + 0.5};
              if (value_of_nodes_[row][col] == 2) {
                  value_of_nodes_[row][col] = 0;
              }
              if (glm::distance(brush_sketchpad_coords, pixel_center) <=
                  brush_radius) {
                  value_of_nodes_[row][col] = 2; //If the distance between the pixels and the
                  // brush is <= the radius then sets that pixel to a shade of 1
                  app.setStartNode(row, col);
                  break;
              }
          }
      }
  }

  void AStarApp::setEndNode(const vec2 &coord) {
      vec2 brush_sketchpad_coords =
              (coord - top_left) / (float) pixel_side_length;
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              vec2 pixel_center = {col + 0.5, row + 0.5};
              if (value_of_nodes_[row][col] == 3) {
                  value_of_nodes_[row][col] = 0;
              }
              if (glm::distance(brush_sketchpad_coords, pixel_center) <=
                  brush_radius) {
                  value_of_nodes_[row][col] = 3; //If the distance between the pixels and the
                  // brush is <= the radius then sets that pixel to a shade of 1
                  app.setEndNode(row, col);
                  break;
              }
          }
      }
  }

  void AStarApp::CreatePath() {
      if (app.getStartNode() != nullptr && app.getEndNode() != nullptr) {
          RemovePath();
          app.SolveAStar();
          Pathfinder::Node *temp = app.getEndNode();
          temp = temp->parent;
          while (temp->parent != nullptr) {
              value_of_nodes_[temp->y_][temp->x_] = 4;
              temp = temp->parent;
          }
      }
      
  }

  void AStarApp::RemovePath() {
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              if (value_of_nodes_[row][col] == 4) {
                  value_of_nodes_[row][col] = 0;
              }
          }
      }
  }


} //namespace visualizer;