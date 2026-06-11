#ifndef NAV_MAP_H
#define NAV_MAP_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 4x4 场地坐标定义：
 *
 * x 从左到右：0, 1, 2, 3
 * y 从上到下：0, 1, 2, 3
 *
 * (0,0) (1,0) (2,0) (3,0)
 * (0,1) (1,1) (2,1) (3,1)
 * (0,2) (1,2) (2,2) (3,2)
 * (0,3) (1,3) (2,3) (3,3)
 */
#define NAV_MAP_W 4
#define NAV_MAP_H 4

/*
 * 方向按顺时针编号：
 * 北=0，东=1，南=2，西=3。
 *
 * 这样反方向可以用 (dir + 2) % 4 得到。
 */
typedef enum {
    DIR_N = 0,
    DIR_E = 1,
    DIR_S = 2,
    DIR_W = 3,
    DIR_NONE = 255
} Direction;

/*
 * 转向动作。
 *
 * 这个不是地图方向，而是“小车为了朝向目标方向，需要怎么转”。
 */
typedef enum {
    TURN_STRAIGHT = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK
} TurnAction;

/*
 * 一个格子的记录。
 *
 * visited：小车是否已经到过这个格子。
 * known：这条边是否已经被传感器确认过。
 * blocked：这条边是否被墙或挡板挡住。
 *
 * known/blocked 都用 4 个 bit 保存：
 * bit0=N, bit1=E, bit2=S, bit3=W。
 */
typedef struct {
    uint8_t visited;
    uint8_t known;
    uint8_t blocked;
} Cell;

/*
 * 整个导航状态。
 *
 * cell：4x4 地图。
 * x/y：小车当前所在格。
 * dir：小车当前车头方向。
 */
typedef struct {
    Cell cell[NAV_MAP_H][NAV_MAP_W];
    uint8_t x;
    uint8_t y;
    Direction dir;
} RobotNav;

/* 初始化地图、小车起点和车头方向。 */
void nav_init(RobotNav *nav);

/* 把小车当前所在格标记为已访问。 */
void nav_mark_current_visited(RobotNav *nav);

/* 判断 4x4 的 16 个格子是否都访问过。 */
bool nav_all_visited(const RobotNav *nav);

/*
 * 设置某个格子的某条边是否 blocked。
 * 会自动同步相邻格子的反方向边。
 */
void nav_set_edge_blocked(RobotNav *nav,
                          uint8_t x,
                          uint8_t y,
                          Direction dir,
                          bool blocked);

/* 设置当前格子的某条边是否 blocked。 */
void nav_set_current_edge_blocked(RobotNav *nav,
                                  Direction dir,
                                  bool blocked);

/* 判断从 (x,y) 往 dir 方向是否能走到相邻格。 */
bool nav_can_go(const RobotNav *nav,
                uint8_t x,
                uint8_t y,
                Direction dir);

/* 小车实际走完一格后，更新软件记录中的 x/y/dir。 */
bool nav_update_position(RobotNav *nav, Direction dir);

/* BFS 找最近未访问格，返回下一步方向。 */
bool nav_plan_to_nearest_unvisited(const RobotNav *nav, Direction *next_dir);

/* BFS 找指定目标格，返回下一步方向。 */
bool nav_plan_to_cell(const RobotNav *nav,
                      uint8_t target_x,
                      uint8_t target_y,
                      Direction *next_dir);

/* 根据当前方向计算左边、右边、反方向。 */
Direction nav_left_of(Direction dir);
Direction nav_right_of(Direction dir);
Direction nav_opposite_dir(Direction dir);

/* 根据当前车头方向和目标方向，判断需要直行/左转/右转/掉头。 */
TurnAction nav_get_turn_action(Direction current, Direction target);

#endif
