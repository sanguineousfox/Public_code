/**
 * @file    build_options.h
 * @brief   Компиляционные режимы стендовой/релизной прошивки.
 */
#ifndef BUILD_OPTIONS_H
#define BUILD_OPTIONS_H

/*
 * 1 — включить диагностический USART2 и все отладочные строки.
 * 0 — полностью убрать вызовы отладочного вывода на этапе компиляции.
 *     USART2 не инициализируется; очередь/строки отладки не занимают
 *     рабочую Flash/RAM после --gc-sections.
 *
 * Для текущей отладки оставлено 1.
 */
#ifndef USART2_DEBUG_ENABLED
#define USART2_DEBUG_ENABLED                1U
#endif

#if ((USART2_DEBUG_ENABLED != 0U) && (USART2_DEBUG_ENABLED != 1U))
#error "USART2_DEBUG_ENABLED must be 0 or 1"
#endif

#endif /* BUILD_OPTIONS_H */
