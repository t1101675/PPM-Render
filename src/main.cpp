#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <json/json.h>
#include <stdlib.h>
#include <time.h>

#include "utils.h"
#include "object.h"
#include "plane.hpp"
#include "ball.hpp"
#include "scene.h"
#include "render.h"

int main(int argc, char* argv[]) {
    //scene
    //read file
    //add objects
    //render
    //print
    std::string filename = "config/test_pic2.json";
    if (argc > 1)
    {
        filename = argv[1];
    }
    std::ifstream fin(filename);
    std::string str;
    Json::Reader reader;
    Json::Value config;
    reader.parse(fin, config);
    Scene scene = Scene();
    scene.load(config["scene"]);

    Render render = Render();
    render.load(config["render"]);
    render.setScene(&scene);
    std::cout << "[info] Render begin" << std::endl;
    render.render();
    std::cout << "[info] Render successed" << std::endl;
    render.save("final");
    return 0;
}