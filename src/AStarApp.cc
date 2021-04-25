//
// Created by Ojas Upalekar on 4/22/21.
//

#include "AStarApp.h"
#include "Pathfinder.h"
namespace visualizer {
  using glm::vec2;
  using pathfinder::Pathfinder;
  AStarApp::AStarApp() {
      ci::app::setWindowSize(kWindowSize, kWindowSize); 
      app = Pathfinder(40,40);
      app.CreateNodes();
      pixel_side_length = (kWindowSize - (2 * kMargin)) / app.getNumOfCols();
  }

  void AStarApp::draw() {
      for (size_t row = 0; row < 40; ++row) {
          for (size_t col = 0; col < 40; ++col) {
              /*if(shade_of_rectangles_[row][col] != 0 ) { //Checks if the shade of the pixel is not 0
                  ci::gl::color(ci::Color::gray(0.3F));
              } else {
                 
              }*/
              ci::gl::color(ci::Color("white"));
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
  }

  void AStarApp::update() {

  }
} //namespace visualizer;