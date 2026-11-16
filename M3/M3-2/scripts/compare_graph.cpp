// 导出 Karto 后端图及已优化扫描点，不读 Gazebo 真值。
#include <karto_sdk/Mapper.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  karto::Mapper mapper;
  mapper.LoadFromFile(argv[1]);
  auto graph = mapper.GetGraph();
  std::ofstream nodes(std::string(argv[2]) + "/nodes.csv");
  std::ofstream edges(std::string(argv[2]) + "/edges.csv");
  std::ofstream points(std::string(argv[2]) + "/points.csv");
  nodes << std::setprecision(15) << "id,time,x,y,yaw\n";
  points << std::setprecision(15) << "id,x,y\n";
  edges << "source,target\n";
  int count = 0;
  for (auto & sensor : graph->GetVertices()) {
    for (auto & item : sensor.second) {
      auto scan = item.second->GetObject();
      const auto p = scan->GetCorrectedPose();
      nodes << scan->GetUniqueId() << ',' << scan->GetTime() << ','
            << p.GetX() << ',' << p.GetY() << ',' << p.GetHeading() << '\n';
      for (const auto & point : scan->GetPointReadings(true))
        points << scan->GetUniqueId() << ',' << point.GetX() << ',' << point.GetY() << '\n';
      ++count;
    }
  }
  for (auto edge : graph->GetEdges())
    edges << edge->GetSource()->GetObject()->GetUniqueId() << ','
          << edge->GetTarget()->GetObject()->GetUniqueId() << '\n';
  std::cout << "VERTICES " << count << "\nEDGES " << graph->GetEdges().size() << '\n';
}
