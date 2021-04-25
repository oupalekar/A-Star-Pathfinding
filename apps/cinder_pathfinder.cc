//
// Created by Ojas Upalekar on 4/22/21.
//
#include "AStarApp.h"
using visualizer::AStarApp;

void prepareSettings(AStarApp::Settings* settings) {
    settings->setResizable(false);
}


CINDER_APP(AStarApp,cinder::app::RendererGl, prepareSettings);