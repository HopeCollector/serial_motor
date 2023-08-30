#include <chrono>
#include <filesystem>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "serial_motor/types/rmd.hpp"

int main(int argc, char** argv) {
  std::string dev("");
  dev = std::string(argv[1]);
  if (dev.empty()) {
    std::cerr << "cannot find motor device" << std::endl;
    exit(-1);
  };

  smotor::rmd m(dev, 1);

  std::vector<double> params;
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      while (i < argc) {
        args.push_back(std::string(argv[i]));
        i++;
      }
    }
  }

  if (args[0] == "-t") {
      m.to(std::stod(args[1]), std::stod(args[2]));
  } else if (args[0] == "-m") {
      m.more(std::stod(args[1]), std::stod(args[2]));
  } else if (args[0] == "-r") {
      m.rotate(std::stod(args[1]));
  } else if (args[0] == "-p") {
    m.pause();
  } else if (args[0] == "-g") {
    // auto start = std::chrono::system_clock::now();
    // for (size_t i = 0; i < 1000; i++)
    // {
    //    std::cout << m.getCurrentPose() << std::endl;
    // }
    // auto end = std::chrono::system_clock::now();
    // auto time = std::chrono::duration<double>(end - start).count();
    // std::cout << "total cost: " << time << std::endl;
    // std::cout << "echo freq: " << 1000 / time << "hz" << std::endl;
    std::cout << m.cur_pose().value_or(0.0) << std::endl;
  }

  return 0;
}
