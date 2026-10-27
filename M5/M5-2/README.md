# M5-2：上位机可视化监控面板

## 项目结构

- robot_side/dashboard_aggregator.py：小车端 ROS2 聚合节点。
- host_side/app.py：主机端 Web 应用。
- host_side/templates/：网页模板。
- host_side/static/：前端脚本和样式。
- config/dashboard.yaml：话题、故障和视频配置。
- tools/measure_resources.py：资源测量工具。
- docs/architecture.md：系统架构。
- docs/fault_rules.md：故障判定规则。
- tests/：自动化测试。

## 环境

小车端使用系统 Python 和 ROS2 Humble。

主机端使用基于系统 Python 创建的独立虚拟环境，并复用系统 ROS2 Python 包。

## 配置

话题匹配规则和监控阈值集中维护在 config/dashboard.yaml。

比赛模式下 video.enable 必须保持 false。

## 运行

完成实现后，运行方式、自动打开浏览器的机制和测试命令将在此处明确说明。
