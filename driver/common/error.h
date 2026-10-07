/**
 * @file    error.h
 * @brief   全工程统一的错误码定义。
 *
 * 所有驱动的公开接口一律返回 error_t，不使用 -1 / -2 这类无意义魔数：
 * 调用者需要能从返回值区分「超时」和「参数非法」是两种完全不同的故障。
 */

#ifndef DRIVER_COMMON_ERROR_H
#define DRIVER_COMMON_ERROR_H

typedef enum {
    ERR_OK = 0,             /**< 成功 */
    ERR_TIMEOUT,            /**< 等待超时 */
    ERR_BUSY,               /**< 设备/总线忙 */
    ERR_INVALID_PARAM,      /**< 入参非法（含空指针、越界） */
    ERR_NOT_INITIALIZED,    /**< 未初始化就调用 */
    ERR_NOT_READY,          /**< 设备尚未就绪 */
    ERR_OVERFLOW,           /**< 溢出 */
    ERR_UNDERFLOW,          /**< 下溢 */
    ERR_COMMUNICATION,      /**< 通信失败（如从机未应答） */
    ERR_NOT_SUPPORTED,      /**< 当前不支持的操作 */
} error_t;

#endif /* DRIVER_COMMON_ERROR_H */
