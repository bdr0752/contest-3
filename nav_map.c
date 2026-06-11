#include "nav_map.h"

/*
 * 方向对应的坐标变化。
 *
 * 北：x 不变，y - 1
 * 东：x + 1，y 不变
 * 南：x 不变，y + 1
 * 西：x - 1，y 不变
 */
static const int8_t g_dx[4] = { 0, 1, 0, -1 };
static const int8_t g_dy[4] = { -1, 0, 1, 0 };

#define DIR_BIT(dir) (1U << (dir))

typedef struct {
    int8_t x;
    int8_t y;
} NavNode;

static bool nav_in_map(int8_t x, int8_t y)
{
    return x >= 0 && x < NAV_MAP_W && y >= 0 && y < NAV_MAP_H;
}

Direction nav_opposite_dir(Direction dir)
{
    return (Direction)((dir + 2) % 4);
}

Direction nav_left_of(Direction dir)
{
    return (Direction)((dir + 3) % 4);
}

Direction nav_right_of(Direction dir)
{
    return (Direction)((dir + 1) % 4);
}

TurnAction nav_get_turn_action(Direction current, Direction target)
{
    uint8_t diff;

    if (current == target) {
        return TURN_STRAIGHT;
    }

    diff = (uint8_t)((target + 4 - current) % 4);

    if (diff == 1) {
        return TURN_RIGHT;
    }
    if (diff == 3) {
        return TURN_LEFT;
    }

    return TURN_BACK;
}

void nav_init(RobotNav *nav)
{
    uint8_t x;
    uint8_t y;

    for (y = 0; y < NAV_MAP_H; y++) {
        for (x = 0; x < NAV_MAP_W; x++) {
            nav->cell[y][x].visited = 0;
            nav->cell[y][x].known = 0;
            nav->cell[y][x].blocked = 0;
        }
    }

    /*
     * 边界墙先写进地图。
     * 这样 BFS 天然不会规划出场地。
     */
    for (x = 0; x < NAV_MAP_W; x++) {
        nav_set_edge_blocked(nav, x, 0, DIR_N, true);
        nav_set_edge_blocked(nav, x, NAV_MAP_H - 1, DIR_S, true);
    }
    for (y = 0; y < NAV_MAP_H; y++) {
        nav_set_edge_blocked(nav, 0, y, DIR_W, true);
        nav_set_edge_blocked(nav, NAV_MAP_W - 1, y, DIR_E, true);
    }

    /*
     * 暂定：小车从 A 口进入后停在 (0,2)，车头朝东。
     * 如果实际入口对应格子不同，只改这里。
     */
    nav->x = 0;
    nav->y = 2;
    nav->dir = DIR_E;
}

void nav_mark_current_visited(RobotNav *nav)
{
    nav->cell[nav->y][nav->x].visited = 1;
}

bool nav_all_visited(const RobotNav *nav)
{
    uint8_t x;
    uint8_t y;

    for (y = 0; y < NAV_MAP_H; y++) {
        for (x = 0; x < NAV_MAP_W; x++) {
            if (!nav->cell[y][x].visited) {
                return false;
            }
        }
    }

    return true;
}

void nav_set_edge_blocked(RobotNav *nav,
                          uint8_t x,
                          uint8_t y,
                          Direction dir,
                          bool blocked)
{
    int8_t nx;
    int8_t ny;
    Direction od;

    if (!nav_in_map((int8_t)x, (int8_t)y) || dir > DIR_W) {
        return;
    }

    nav->cell[y][x].known |= DIR_BIT(dir);

    if (blocked) {
        nav->cell[y][x].blocked |= DIR_BIT(dir);
    } else {
        nav->cell[y][x].blocked &= (uint8_t)~DIR_BIT(dir);
    }

    nx = (int8_t)x + g_dx[dir];
    ny = (int8_t)y + g_dy[dir];

    if (nav_in_map(nx, ny)) {
        od = nav_opposite_dir(dir);
        nav->cell[ny][nx].known |= DIR_BIT(od);

        if (blocked) {
            nav->cell[ny][nx].blocked |= DIR_BIT(od);
        } else {
            nav->cell[ny][nx].blocked &= (uint8_t)~DIR_BIT(od);
        }
    }
}

void nav_set_current_edge_blocked(RobotNav *nav,
                                  Direction dir,
                                  bool blocked)
{
    nav_set_edge_blocked(nav, nav->x, nav->y, dir, blocked);
}

bool nav_can_go(const RobotNav *nav,
                uint8_t x,
                uint8_t y,
                Direction dir)
{
    int8_t nx;
    int8_t ny;

    if (!nav_in_map((int8_t)x, (int8_t)y) || dir > DIR_W) {
        return false;
    }

    nx = (int8_t)x + g_dx[dir];
    ny = (int8_t)y + g_dy[dir];

    if (!nav_in_map(nx, ny)) {
        return false;
    }

    if (nav->cell[y][x].blocked & DIR_BIT(dir)) {
        return false;
    }

    return true;
}

bool nav_update_position(RobotNav *nav, Direction dir)
{
    int8_t nx;
    int8_t ny;

    if (!nav_can_go(nav, nav->x, nav->y, dir)) {
        return false;
    }

    nx = (int8_t)nav->x + g_dx[dir];
    ny = (int8_t)nav->y + g_dy[dir];

    nav->x = (uint8_t)nx;
    nav->y = (uint8_t)ny;
    nav->dir = dir;

    return true;
}

/*
 * BFS 公共函数。
 *
 * 如果 target_x/target_y 在地图内，就规划到指定格子。
 * 如果 target_x/target_y 是 -1，就规划到最近未访问格。
 */
static bool nav_bfs_first_step(const RobotNav *nav,
                               int8_t target_x,
                               int8_t target_y,
                               Direction *next_dir)
{
    bool used[NAV_MAP_H][NAV_MAP_W] = { false };
    int8_t parent_x[NAV_MAP_H][NAV_MAP_W];
    int8_t parent_y[NAV_MAP_H][NAV_MAP_W];
    NavNode queue[NAV_MAP_W * NAV_MAP_H];
    int head = 0;
    int tail = 0;
    int8_t found_x = -1;
    int8_t found_y = -1;
    uint8_t x;
    uint8_t y;

    for (y = 0; y < NAV_MAP_H; y++) {
        for (x = 0; x < NAV_MAP_W; x++) {
            parent_x[y][x] = -1;
            parent_y[y][x] = -1;
        }
    }

    queue[tail].x = (int8_t)nav->x;
    queue[tail].y = (int8_t)nav->y;
    tail++;
    used[nav->y][nav->x] = true;

    while (head < tail) {
        NavNode cur = queue[head];
        head++;

        if (target_x >= 0 && target_y >= 0) {
            if (cur.x == target_x && cur.y == target_y) {
                found_x = cur.x;
                found_y = cur.y;
                break;
            }
        } else {
            if (!(cur.x == nav->x && cur.y == nav->y) &&
                !nav->cell[cur.y][cur.x].visited) {
                found_x = cur.x;
                found_y = cur.y;
                break;
            }
        }

        for (Direction dir = DIR_N; dir <= DIR_W; dir++) {
            int8_t nx;
            int8_t ny;

            if (!nav_can_go(nav, (uint8_t)cur.x, (uint8_t)cur.y, dir)) {
                continue;
            }

            nx = cur.x + g_dx[dir];
            ny = cur.y + g_dy[dir];

            if (used[ny][nx]) {
                continue;
            }

            used[ny][nx] = true;
            parent_x[ny][nx] = cur.x;
            parent_y[ny][nx] = cur.y;

            queue[tail].x = nx;
            queue[tail].y = ny;
            tail++;
        }
    }

    if (found_x < 0) {
        *next_dir = DIR_NONE;
        return false;
    }

    /*
     * 如果目标就是当前格，说明已经到了，不需要移动。
     */
    if (found_x == nav->x && found_y == nav->y) {
        *next_dir = DIR_NONE;
        return true;
    }

    /*
     * 从目标格根据 parent 往回倒推，直到找到“当前格之后的第一格”。
     */
    while (!(parent_x[found_y][found_x] == nav->x &&
             parent_y[found_y][found_x] == nav->y)) {
        int8_t px = parent_x[found_y][found_x];
        int8_t py = parent_y[found_y][found_x];

        if (px < 0 || py < 0) {
            *next_dir = DIR_NONE;
            return false;
        }

        found_x = px;
        found_y = py;
    }

    if (found_x > nav->x) {
        *next_dir = DIR_E;
    } else if (found_x < nav->x) {
        *next_dir = DIR_W;
    } else if (found_y > nav->y) {
        *next_dir = DIR_S;
    } else if (found_y < nav->y) {
        *next_dir = DIR_N;
    } else {
        *next_dir = DIR_NONE;
    }

    return *next_dir != DIR_NONE;
}

bool nav_plan_to_nearest_unvisited(const RobotNav *nav, Direction *next_dir)
{
    return nav_bfs_first_step(nav, -1, -1, next_dir);
}

bool nav_plan_to_cell(const RobotNav *nav,
                      uint8_t target_x,
                      uint8_t target_y,
                      Direction *next_dir)
{
    if (!nav_in_map((int8_t)target_x, (int8_t)target_y)) {
        *next_dir = DIR_NONE;
        return false;
    }

    return nav_bfs_first_step(nav, (int8_t)target_x, (int8_t)target_y, next_dir);
}
