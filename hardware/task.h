#ifndef TASK_H
#define TASK_H

/*
 * task.h 是任务层对外的接口。
 *
 * 其他文件如果想启动某个比赛任务，只需要包含这个头文件，
 * 然后调用 task_1_go_from_a_to_c() 或 task_2_traverse_16_cells_then_exit_c()。
 */

typedef enum {
    TASK_OK = 0,           /* 任务正常完成 */
    TASK_NO_PATH,          /* BFS 找不到可走路径，或前方临时发现挡板 */
    TASK_MOVE_FAILED,      /* 转向/前进/停车等底层动作失败 */
    TASK_TOO_MANY_STEPS    /* 步数超过保护上限，防止程序一直绕圈 */
} TaskResult;

/*
 * 题目1：
 * 无随机挡板，从 A 口进入，最后从 C 口驶出。
 */
TaskResult task_1_go_from_a_to_c(void);

/*
 * 题目2：
 * 随机两块挡板，遍历 16 个格子，然后从 C 口驶出。
 */
TaskResult task_2_traverse_16_cells_then_exit_c(void);

#endif
