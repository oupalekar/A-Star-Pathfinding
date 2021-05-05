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
      ci::app::setWindowSize(1.5 * kWindowSize, kWindowSize);
      app = Pathfinder(num_of_nodes_side, num_of_nodes_side);
      //app.setDiagonals(true);
      app.CreateNodes();
      value_of_nodes_ = vector<vector<size_t>>(num_of_nodes_side, vector<size_t>(num_of_nodes_side, 0));
  }

  void AStarApp::draw() {
      DrawConnections();
      for (size_t row = 0; row < app.getNumOfRows(); ++row) {
          for (size_t col = 0; col < app.getNumOfCols(); ++col) {
              if(&app.getArrayOfNodes()[row * app.getNumOfCols() + col] == app.getStartNode()) {
                  ci::gl::color(ci::Color("green"));
              } else if (&app.getArrayOfNodes()[row * app.getNumOfCols() + col] == app.getEndNode()) {
                  ci::gl::color(ci::Color("red"));
              } else if (app.getArrayOfNodes()[row * app.getNumOfCols() + col].is_obstacle_) {
                  ci::gl::color(ci::Color::gray(0.3F));
              } else if (app.getArrayOfNodes()[row * app.getNumOfCols() + col].is_visited_) {
                  ci::gl::color(ci::Color8u(173, 216, 230));
              } else {
                  ci::gl::color(ci::Color("blue"));
              }
              
              vec2 pixel_top_left = top_left + vec2(col * pixel_side_length,
                                                    row * pixel_side_length) 
                                                            + (vec2(kMargin, 0) * vec2(col, 0)) 
                                                            + (vec2( 0, kMargin) * vec2(0, row));

              vec2 pixel_bottom_right =
                      pixel_top_left + vec2(pixel_side_length, pixel_side_length);
              ci::Rectf pixel_bounding_box(pixel_top_left, pixel_bottom_right);

              ci::gl::drawSolidRect(pixel_bounding_box);

              ci::gl::color(ci::Color("black"));
              ci::gl::drawStrokedRect(pixel_bounding_box);
          }
      }
      CreatePath();
  }

  void AStarApp::mouseDown(ci::app::MouseEvent event) {
      if (event.isControlDown()) {
          setStartNode(event.getPos());
      } else if (event.isShiftDown() && app.getStartNode() != nullptr) {
          setEndNode(event.getPos());
      } else {
          if (event.isRight()) {
              updateBoard(event.getPos(), 0);
          } else {
              updateBoard(event.getPos(), 1);
          }
      }
  }

  void AStarApp::mouseDrag(ci::app::MouseEvent event) {
      if (event.isRight()) {
          updateBoard(event.getPos(), 0);
      } else {
          updateBoard(event.getPos(), 1);
      }
  }

  void AStarApp::updateBoard(const vec2 &coord, size_t value) {
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              vec2 top_length_node = top_left + vec2(col * pixel_side_length,row * pixel_side_length)
                                                + (vec2(kMargin, 0) * vec2(col, 0))
                                                + (vec2( 0, kMargin) * vec2(0, row));
              vec2 pixel_center = top_length_node + vec2(pixel_side_length/2, pixel_side_length/2);
              if (glm::distance(coord, pixel_center) <=
                  radius) {
                  if(&app.getArrayOfNodes()[row * app.getNumOfCols() + col] == app.getStartNode()) {
                      app.setStartNode(-1,-1);
                  }
                  if(&app.getArrayOfNodes()[row * app.getNumOfCols() + col] == app.getEndNode()) {
                      app.setEndNode(-1,-1);
                  }
                  if (value == 1) {
                      app.setObstacle(col, row, true);
                  } else {
                      app.setObstacle(col, row, false);
                  }
              }
          }
      }
  }

  void AStarApp::setStartNode(const vec2 &coord) {
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              vec2 top_length_node = top_left + vec2(col * pixel_side_length,row * pixel_side_length)
                                     + (vec2(kMargin, 0) * vec2(col, 0))
                                     + (vec2( 0, kMargin) * vec2(0, row));
              vec2 pixel_center = top_length_node + vec2(pixel_side_length/2, pixel_side_length/2);
              if (glm::distance(coord, pixel_center) <=
                  radius) {
                  app.setStartNode(row, col);
                  break;
              }
          }
      }
  }

  void AStarApp::setEndNode(const vec2 &coord) {
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              vec2 top_length_node = top_left + vec2(col * pixel_side_length,row * pixel_side_length)
                                     + (vec2(kMargin, 0) * vec2(col, 0))
                                     + (vec2( 0, kMargin) * vec2(0, row));
              vec2 pixel_center = top_length_node + vec2(pixel_side_length/2, pixel_side_length/2);
              if (glm::distance(coord, pixel_center) <=
                  radius) {
                  app.setEndNode(row, col);
                  break;
              }
          }
      }
  }

  void AStarApp::CreatePath() {
      if (app.getStartNode() != nullptr && app.getEndNode() != nullptr) {
          app.SolveAStar();
          Pathfinder::Node *temp = app.getEndNode();
          //temp = temp->parent;
          while (temp->parent != nullptr) {

              vec2 pixel_top_left = top_left + vec2(temp->x_ * pixel_side_length,
                                                    temp->y_ * pixel_side_length)
                                    + (vec2(kMargin, 0) * vec2(temp->x_, 0))
                                    + (vec2( 0, kMargin) * vec2(0, temp->y_));
              vec2 pixel_center = pixel_top_left + vec2(pixel_side_length/2, pixel_side_length/2);
              vec2 parent_pixel_top_left = top_left + vec2(temp->parent->x_ * pixel_side_length,
                                                    temp->parent->y_ * pixel_side_length)
                                    + (vec2(kMargin, 0) * vec2(temp->parent->x_, 0))
                                    + (vec2( 0, kMargin) * vec2(0, temp->parent->y_));
              vec2 parent_pixel_center = parent_pixel_top_left + vec2(pixel_side_length/2, pixel_side_length/2);
              ci::gl::color(ci::Color("red"));
              ci::gl::drawLine(pixel_center, parent_pixel_center);
              temp = temp->parent;
          }
      }
  }

  void AStarApp::DrawConnections() {
      for (size_t row = 0; row < app.getNumOfRows(); row++) {
          for (size_t col = 0; col < app.getNumOfCols(); col++) {
              for (Pathfinder::Node *n : app.getArrayOfNodes()[row * app.getNumOfCols() + col].neighbors_) {
                  Pathfinder::Node* main = &app.getArrayOfNodes()[row * app.getNumOfCols() + col];
                  vec2 pixel_top_left = top_left + vec2(main->x_ * pixel_side_length,
                                                        main->y_ * pixel_side_length)
                                        + (vec2(kMargin, 0) * vec2(main->x_, 0))
                                        + (vec2( 0, kMargin) * vec2(0, main->y_));
                  vec2 pixel_center = pixel_top_left + vec2(pixel_side_length/2, pixel_side_length/2);
                  vec2 parent_pixel_top_left = top_left + vec2(n->x_ * pixel_side_length,
                                                               n->y_ * pixel_side_length)
                                               + (vec2(kMargin, 0) * vec2(n->x_, 0))
                                               + (vec2( 0, kMargin) * vec2(0, n->y_));
                  vec2 parent_pixel_center = parent_pixel_top_left + vec2(pixel_side_length/2, pixel_side_length/2);
                  ci::gl::lineWidth(1000);
                  ci::gl::color(ci::Color("blue"));
                  ci::gl::drawLine(pixel_center, parent_pixel_center);
              }
          }
      }
  }


} //namespace visualizer;


/*if (value_of_nodes_[row][col] == 1) { //Checks if the shade of the pixel is not 0
                  ci::gl::color(ci::Color::gray(0.3F));
              } else if (value_of_nodes_[row][col] == 2) {
                  ci::gl::color(ci::Color("green"));
              } else if (value_of_nodes_[row][col] == 3) {
                  ci::gl::color(ci::Color("red"));
              } else if (value_of_nodes_[row][col] == 4) {
                  ci::gl::color(ci::Color("yellow"));
              } else if (value_of_nodes_[row][col] == 5) {
                      ci::gl::color(ci::Color("blue"));
              } else {
                  ci::gl::color(ci::Color("white"));
              }*/
/*
 * switch (value_of_nodes_[row][col]) {
    case 1:
        ci::gl::color(ci::Color::gray(0.3F));
    case 2: 
        ci::gl::color(ci::Color("green"));
    case 3:
        ci::gl::color(ci::Color("red"));
    case 4: 
        ci::gl::color(ci::Color("yellow"));
    case 0:
        ci::gl::color(ci::Color("white"));
}
 */


