#include "task.h"
#include "nav_map.h"

/*
 * 这个文件只放“任务流程”。
 *
 * nav_map.c 负责：
 *   1. 维护 4x4 地图
 *   2. 记录哪些格子访问过
 *   3. 记录哪些边被挡板/墙挡住
 *   4. 用 BFS 规划下一步方向
 *
 * taskc.c 负责：
 *   1. 任务1：从 A 进，去 C 出
 *   2. 任务2：遍历 16 格，再去 C 出
 *   3. 把“规划出来的方向”转换成“转向 + 走一格”
 *
 * 目前这里的 motion_* 和 sensor_* 函数是占位版本。
 * 后面接真实底层时，把它们替换成电机、IMU、ToF 的代码即可。
 */

#define EXIT_C_X 3
#define EXIT_C_Y 1
#define TASK_MAX_STEPS 80

/*
 * ============================
 * 下面是硬件占位接口
 * ============================
 *
 * 这些函数现在只是为了让任务逻辑能读懂。
 * 真实小车里应该这样替换：
 *
 * motion_turn_left_90()       -> IMU 闭环左转 90 度
 * motion_turn_right_90()      -> IMU 闭环右转 90 度
 * motion_turn_back_180()      -> 两次 90 度或 180 度闭环转向
 * motion_move_one_cell()      -> 编码器距离闭环 + IMU 航向保持，前进 45cm
 * sensor_front_blocked()      -> 前 ToF 判断前方边是否有墙/挡板
 * sensor_left_blocked()       -> 左 ToF 判断左侧边是否有墙/挡板
 * sensor_right_blocked()      -> 右 ToF 判断右侧边是否有墙/挡板
 */

static bool motion_turn_left_90(void)
{
    /*
     * TODO:
     * 后续替换成真实 IMU 闭环左转 90 度。
     */
    return true;
}

static bool motion_turn_right_90(void)
{
    /*
     * TODO:
     * 后续替换成真实 IMU 闭环右转 90 度。
     */
    return true;
}

static bool motion_turn_back_180(void)
{
    /*
     * TODO:
     * 可以连续调用两次 90 度转向，也可以单独做 180 度闭环。
     */
    return true;
}

static bool motion_move_one_cell(void)
{
    /*
     * TODO:
     * 后续替换成“编码器距离闭环 + IMU 航向保持”。
     * 目标距离约为一个格子，即 45cm。
     */
    return true;
}

static bool motion_enter_map_from_a(void)
{
    /*
     * TODO:
     * 这里代表“从准备区启动后，穿过 A 口并走到地图起始格中心”。
     *
     * 后续真实实现时，这里不一定正好等于一个标准格长。
     * 它更像一个专门的入图动作，距离需要靠实车标定。
     *
     * 推荐真实版本做法：
     * 1. 启动后直行进入 A 口
     * 2. 走到约定好的起始格中心
     * 3. 停车
     * 4. 必要时做一次姿态校正
     */
    return true;
}

static void motion_stop(void)
{
    /*
     * TODO:
     * 后续替换成电机驱动板停车命令。
     */
}

static bool sensor_front_blocked(void)
{
    /*
     * TODO:
     * 后续替换成前 ToF 测距判断。
     * true 表示前方边有墙/挡板。
     */
    return false;
}

static bool sensor_left_blocked(void)
{
    /*
     * TODO:
     * 后续替换成左 ToF 测距判断。
     */
    return false;
}

static bool sensor_right_blocked(void)
{
    /*
     * TODO:
     * 后续替换成右 ToF 测距判断。
     */
    return false;
}

/*
 * 根据小车当前朝向，把前/左/右 ToF 的检测结果写进地图。
 *
 * 例子：
 *   小车当前朝东。
 *   前方 ToF 检测到挡板 -> 当前格子的东边 blocked。
 *   左侧 ToF 检测到挡板 -> 当前格子的北边 blocked。
 *   右侧 ToF 检测到挡板 -> 当前格子的南边 blocked。
 */
static void task_sense_edges_at_current_cell(RobotNav *nav)
{
    Direction front = nav->dir;
    Direction left = nav_left_of(nav->dir);
    Direction right = nav_right_of(nav->dir);

    nav_set_current_edge_blocked(nav, front, sensor_front_blocked());
    nav_set_current_edge_blocked(nav, left, sensor_left_blocked());
    nav_set_current_edge_blocked(nav, right, sensor_right_blocked());
}

/*
 * 把车头转到目标方向。
 *
 * 导航层只知道“下一步要往 DIR_E 走”。
 * 执行层要负责判断现在车头朝哪，然后决定左转、右转、掉头还是不转。
 */
static bool task_turn_to_direction(RobotNav *nav, Direction target_dir)
{
    TurnAction action = nav_get_turn_action(nav->dir, target_dir);

    if (action == TURN_STRAIGHT) {
        nav->dir = target_dir;
        return true;
    }

    if (action == TURN_LEFT) {
        if (!motion_turn_left_90()) {
            return false;
        }
        nav->dir = target_dir;
        return true;
    }

    if (action == TURN_RIGHT) {
        if (!motion_turn_right_90()) {
            return false;
        }
        nav->dir = target_dir;
        return true;
    }

    if (!motion_turn_back_180()) {
        return false;
    }
    nav->dir = target_dir;
    return true;
}

/*
 * 执行“往 next_dir 方向走一格”。
 *
 * 流程：
 *   1. 先转向 next_dir。
 *   2. 转完后用前 ToF 再确认前方是否突然有挡板。
 *   3. 如果有挡板，写进地图，不走。
 *   4. 如果没挡板，调用运动层前进 45cm。
 *   5. 走完后更新 nav->x/nav->y。
 */
static TaskResult task_go_one_cell(RobotNav *nav, Direction next_dir)
{
    if (!task_turn_to_direction(nav, next_dir)) {
        motion_stop();
        return TASK_MOVE_FAILED;
    }

    if (sensor_front_blocked()) {
        nav_set_current_edge_blocked(nav, nav->dir, true);
        motion_stop();
        return TASK_NO_PATH;
    }

    nav_set_current_edge_blocked(nav, nav->dir, false);

    if (!motion_move_one_cell()) {
        motion_stop();
        return TASK_MOVE_FAILED;
    }

    if (!nav_update_position(nav, next_dir)) {
        motion_stop();
        return TASK_MOVE_FAILED;
    }

    nav_mark_current_visited(nav);
    return TASK_OK;
}

/*
 * 从准备区进入地图。
 *
 * 当前整个 BFS 导航都是从“已经进入 4x4 地图的第一个起始格”开始算的。
 * 所以在真正开始任务前，需要先做一次入图动作。
 *
 * 当前仍然采用之前的坐标假设：
 *   入图完成后，小车位于 (0,2)，车头朝东。
 *
 * 注意：
 *   nav_init() 只是初始化软件里的地图和起始坐标；
 *   真正把车开到这个位置的是 motion_enter_map_from_a()。
 */
static TaskResult task_enter_map_from_a(RobotNav *nav)
{
    nav_init(nav);

    if (!motion_enter_map_from_a()) {
        motion_stop();
        return TASK_MOVE_FAILED;
    }

    /*
     * 入图完成后，逻辑上认为车已经到达起始格中心。
     */
    nav_mark_current_visited(nav);
    return TASK_OK;
}

/*
 * 规划到指定格子，并一步一步走过去。
 *
 * 任务1去 C 口前，可以用它走到 C 对应的格子。
 * 任务2遍历完成后，也可以复用它走到 C。
 */
static TaskResult task_go_to_cell(RobotNav *nav, uint8_t target_x, uint8_t target_y)
{
    uint8_t steps = 0;

    while (!(nav->x == target_x && nav->y == target_y)) {
        Direction next_dir = DIR_NONE;
        TaskResult move_result;

        task_sense_edges_at_current_cell(nav);

        if (!nav_plan_to_cell(nav, target_x, target_y, &next_dir)) {
            motion_stop();
            return TASK_NO_PATH;
        }

        move_result = task_go_one_cell(nav, next_dir);
        if (move_result == TASK_NO_PATH) {
            continue;
        }
        if (move_result != TASK_OK) {
            return move_result;
        }

        steps++;
        if (steps > TASK_MAX_STEPS) {
            motion_stop();
            return TASK_TOO_MANY_STEPS;
        }
    }

    return TASK_OK;
}

/*
 * 从 C 口驶出。
 *
 * 暂定 C 口在 (3,1) 的东边。
 * 所以到达 (3,1) 后，车头转向东，再往前走一格的距离驶出场地。
 */
static TaskResult task_exit_from_c(RobotNav *nav)
{
    TaskResult result;

    result = task_go_to_cell(nav, EXIT_C_X, EXIT_C_Y);
    if (result != TASK_OK) {
        return result;
    }

    if (!task_turn_to_direction(nav, DIR_E)) {
        motion_stop();
        return TASK_MOVE_FAILED;
    }

    /*
     * 注意：这里不调用 nav_update_position()。
     * 因为驶出 C 口后已经不在 4x4 地图里面了。
     */
    if (!motion_move_one_cell()) {
        motion_stop();
        return TASK_MOVE_FAILED;
    }

    motion_stop();
    return TASK_OK;
}

/*
 * 任务1：无随机挡板，从 A 进入，任选路径，从 C 驶出。
 *
 * 第一版直接规划到 C 对应格子，再朝东驶出。
 */
TaskResult task_1_go_from_a_to_c(void)
{
    RobotNav nav;
    TaskResult result;

    result = task_enter_map_from_a(&nav);
    if (result != TASK_OK) {
        return result;
    }

    return task_exit_from_c(&nav);
}

/*
 * 任务2：随机两块挡板，遍历 16 格，然后从 C 驶出。
 *
 * 核心循环：
 *   1. 停在当前格中心。
 *   2. ToF 扫前/左/右，把挡板写进地图。
 *   3. BFS 找最近未访问格。
 *   4. 朝 BFS 给出的 next_dir 走一格。
 *   5. 重复直到 16 格全部 visited。
 */
TaskResult task_2_traverse_16_cells_then_exit_c(void)
{
    RobotNav nav;
    uint8_t steps = 0;
    TaskResult result;

    result = task_enter_map_from_a(&nav);
    if (result != TASK_OK) {
        return result;
    }

    while (!nav_all_visited(&nav)) {
        Direction next_dir = DIR_NONE;
        TaskResult move_result;

        task_sense_edges_at_current_cell(&nav);

        if (!nav_plan_to_nearest_unvisited(&nav, &next_dir)) {
            motion_stop();
            return TASK_NO_PATH;
        }

        move_result = task_go_one_cell(&nav, next_dir);

        /*
         * 如果准备走之前发现前方有挡板，task_go_one_cell()
         * 会把这条边记录进地图并返回 TASK_NO_PATH。
         * 此时不要报错，重新 BFS 规划即可。
         */
        if (move_result == TASK_NO_PATH) {
            continue;
        }

        if (move_result != TASK_OK) {
            return move_result;
        }

        steps++;
        if (steps > TASK_MAX_STEPS) {
            motion_stop();
            return TASK_TOO_MANY_STEPS;
        }
    }

    return task_exit_from_c(&nav);
}
