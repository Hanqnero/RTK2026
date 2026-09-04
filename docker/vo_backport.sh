#!/usr/bin/env bash
# Backport Vector Object Server из Nav2 в ветку jazzy.
#
# Зачем
# -----
#
# vector_object_server появился в Nav2 только в main (1.5.0). В jazzy (1.3.13)
# и даже в kilted (1.4.2) его нет, отдельного пакета в apt тоже нет.
# rtk2026_vector_objects написан против него: keepout_click_tool импортирует
# nav2_msgs.msg.PolygonObject и nav2_msgs.srv.AddShapes, которых в Jazzy не
# существует, поэтому нода без backport даже не импортируется.
#
# Почему не взять ветку целиком
# -----------------------------
#
# Донор grupo-avispa:vo основан на 1.3.1, и в его nav2_msgs НЕТ интерфейсов
# nav2_route: ComputeRoute, Route, RouteEdge, RouteNode, SetRouteGraph,
# DynamicEdges. Заменив ими системные, мы получили бы vector objects ценой
# маршрутизации по городу - то есть сломали бы ровно то, ради чего карта и
# правится. Ветку main взять тоже нельзя: она требует пакет nav2_ros_common,
# которого в Jazzy нет вовсе.
#
# Поэтому здесь точечный перенос: за основу берётся jazzy, из донора
# добавляются только пять интерфейсов и реализация сервера.
#
# Результат в /opt/vo_ws. Сорсить ПОСЛЕ /opt/ros/jazzy и ДО /workspace.

set -euo pipefail

SRC=/opt/vo_src
WS=/opt/vo_ws
BASE_BRANCH="${BASE_BRANCH:-jazzy}"
DONOR_REPO="${DONOR_REPO:-https://github.com/grupo-avispa/navigation2.git}"
DONOR_BRANCH="${DONOR_BRANCH:-vo}"

rm -rf "$SRC" "$WS"
mkdir -p "$SRC"
cd "$SRC"

git clone --depth 1 -b "$BASE_BRANCH" https://github.com/ros-navigation/navigation2.git nav2
git clone --depth 1 -b "$DONOR_BRANCH" "$DONOR_REPO" vo

MS=nav2/nav2_map_server
INC=$MS/include/nav2_map_server

# 1. Интерфейсы, которых нет в Jazzy.
for f in msg/CircleObject.msg msg/PolygonObject.msg \
         srv/AddShapes.srv srv/GetShapes.srv srv/RemoveShapes.srv; do
    cp "vo/nav2_msgs/$f" "nav2/nav2_msgs/$f"
done

# 2. Реализация сервера.
cp vo/$MS/../nav2_map_server/include/nav2_map_server/vector_object_*.hpp "$INC/" 2>/dev/null || \
cp vo/nav2_map_server/include/nav2_map_server/vector_object_*.hpp "$INC/"
mkdir -p "$MS/src/vo_server"
cp vo/nav2_map_server/src/vo_server/*.cpp "$MS/src/vo_server/"

# 3. Три заголовка nav2_util, которых нет в Jazzy.
#
# Кладутся под nav2_map_server, а не под nav2_util: install(DIRECTORY include/)
# положил бы их в системный путь и перекрыл настоящий nav2_util, чей .so
# собран с другими версиями остальных заголовков.
for h in occ_grid_utils polygon_utils raytrace_line_2d; do
    cp "vo/nav2_util/include/nav2_util/$h.hpp" "$INC/$h.hpp"
done
sed -i -E 's@#include "nav2_util/(occ_grid_utils|polygon_utils|raytrace_line_2d)\.hpp"@#include "nav2_map_server/\1.hpp"@g' \
    "$MS/src/vo_server"/*.cpp "$INC"/vector_object_*.hpp "$INC"/occ_grid_utils.hpp \
    "$INC"/polygon_utils.hpp "$INC"/raytrace_line_2d.hpp

python3 - "$SRC" <<'PY'
import pathlib, sys
root = pathlib.Path(sys.argv[1]) / "nav2"

# nav2_msgs: зарегистрировать пять новых интерфейсов
p = root / "nav2_msgs/CMakeLists.txt"
s = p.read_text()
if "CircleObject.msg" not in s:
    anchor = '  "msg/Route.msg"'
    assert anchor in s, "якорь msg/Route.msg не найден"
    add = "\n".join(f'  "{x}"' for x in (
        "msg/CircleObject.msg", "msg/PolygonObject.msg",
        "srv/AddShapes.srv", "srv/GetShapes.srv", "srv/RemoveShapes.srv"))
    p.write_text(s.replace(anchor, add + "\n" + anchor, 1))

# nav2_map_server: цели сборки в стиле ветки jazzy
p = root / "nav2_map_server/CMakeLists.txt"
s = p.read_text()
if "vo_server_executable" not in s:
    s = s.replace("find_package(tf2 REQUIRED)",
        "find_package(tf2 REQUIRED)\nfind_package(tf2_ros REQUIRED)\n"
        "find_package(geometry_msgs REQUIRED)\nfind_package(lifecycle_msgs REQUIRED)", 1)
    s = s.replace("set(costmap_filter_info_server_executable costmap_filter_info_server)",
        "set(costmap_filter_info_server_executable costmap_filter_info_server)\n\n"
        "set(vo_library_name vector_object_core)\n\n"
        "set(vo_server_executable vector_object_server)", 1)
    s = s.replace("set(map_io_library_name map_io)",
        "add_executable(${vo_server_executable}\n  src/vo_server/main.cpp)\n\n"
        "set(map_io_library_name map_io)", 1)
    s = s.replace("set(map_io_dependencies",
        "add_library(${vo_library_name} SHARED\n"
        "  src/vo_server/vector_object_shapes.cpp\n"
        "  src/vo_server/vector_object_server.cpp)\n\n"
        "set(vo_dependencies\n  rclcpp\n  rclcpp_lifecycle\n  rclcpp_components\n"
        "  nav_msgs\n  nav2_msgs\n  nav2_util\n  geometry_msgs\n  lifecycle_msgs\n"
        "  std_msgs\n  tf2\n  tf2_ros)\n\nset(map_io_dependencies", 1)
    s = s.replace(
        "ament_target_dependencies(${map_io_library_name}\n  ${map_io_dependencies})",
        "ament_target_dependencies(${map_io_library_name}\n  ${map_io_dependencies})\n\n"
        "ament_target_dependencies(${vo_library_name}\n  ${vo_dependencies})\n\n"
        "ament_target_dependencies(${vo_server_executable}\n  ${vo_dependencies})\n\n"
        "target_link_libraries(${vo_server_executable}\n  ${vo_library_name})", 1)
    s = s.replace(
        'rclcpp_components_register_nodes(${library_name} "nav2_map_server::MapServer")',
        'rclcpp_components_register_nodes(${library_name} "nav2_map_server::MapServer")\n'
        'rclcpp_components_register_nodes(${vo_library_name} "nav2_map_server::VectorObjectServer")', 1)
    s = s.replace("    ${library_name} ${map_io_library_name}\n",
                  "    ${library_name} ${map_io_library_name} ${vo_library_name}\n", 1)
    s = s.replace("    ${costmap_filter_info_server_executable}\n",
                  "    ${costmap_filter_info_server_executable} ${vo_server_executable}\n", 1)
    p.write_text(s)

# package.xml: зависимости, которых нет в jazzy-версии
p = root / "nav2_map_server/package.xml"
t = p.read_text()
for dep in ("geometry_msgs", "lifecycle_msgs", "tf2_ros"):
    if f"<depend>{dep}</depend>" not in t:
        t = t.replace("  <depend>tf2</depend>",
                      f"  <depend>tf2</depend>\n  <depend>{dep}</depend>", 1)
p.write_text(t)
print("патчи применены")
PY

mkdir -p "$WS/src"
ln -sfn "$SRC/nav2/nav2_msgs"       "$WS/src/nav2_msgs"
ln -sfn "$SRC/nav2/nav2_map_server" "$WS/src/nav2_map_server"

# ROS-скрипты обращаются к необъявленным переменным, при set -u это падение.
set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "$WS"
colcon build --packages-select nav2_msgs nav2_map_server \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

test -x "$WS/install/nav2_map_server/lib/nav2_map_server/vector_object_server" \
    || { echo "vector_object_server не собрался" >&2; exit 1; }

rm -rf "$WS/build" "$WS/log"
echo "backport готов: $WS"
