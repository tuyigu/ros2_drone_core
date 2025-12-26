# 🛠️ 工程自动化工具集 (Developer Tools)

这个目录存放了用于提升 ROS 2 开发效率的辅助脚本。

## 1. `create_pkg.sh` - 项目脚手架

### 💡 背景
在 ROS 2 开发中，手动配置 `CMakeLists.txt` 和 `package.xml` 极易出错且浪费时间。

### ✨ 功能
- **标准化目录**: 自动创建 `include`, `src`, `config`, `launch` 文件夹。
- **集成编译数据库**: 自动开启 `CMAKE_EXPORT_COMPILE_COMMANDS`，确保 CLion/VSCode 补全顺滑。
- **预设依赖**: 默认包含 `rclcpp` 和 `px4_msgs`，适合无人机开发。

### 🚀 使用方法
在工作空间的根目录下运行：
\`\`\`bash
./ros2_drone_core/scripts/create_pkg.sh
\`\`\`

### ⚙️ 自定义
修改脚本顶部的 `PKG_NAME` 和 `NODE_NAME` 变量即可快速生成新项目。