/**
 * @file    build_options.h
 * @brief   Компиляционные режимы стендовой/релизной прошивки.
 *
 * ВНИМАНИЕ:
 * EMERGENCY_SINGLE_PULSE_DEBUG_MODE предназначен только для аварийной
 * стендовой отладки при неисправной второй приемной катушке/ее проводке.
 * Перед релизом обязательно установить 0.
 */
#ifndef BUILD_OPTIONS_H
#define BUILD_OPTIONS_H

/*
 * 1 — аварийный одноканальный режим:
 *     - измерение завершается по первому валидному фронту t1;
 *     - второй импульс не ожидается и интервал пары не проверяется;
 *     - счетчик PAIR/1P и авария катушки не влияют на измерение;
 *     - команды 01/02/11/12/13 калибруются по свежим одиночным t1.
 * 0 — штатный режим с обязательной логикой контроля пары.
 *
 * ТЕКУЩЕЕ СТЕНДОВОЕ ЗНАЧЕНИЕ: 1.
 * ДЛЯ РЕЛИЗА: 0.
 */
#ifndef EMERGENCY_SINGLE_PULSE_DEBUG_MODE
#define EMERGENCY_SINGLE_PULSE_DEBUG_MODE  1U
#endif

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

#if ((EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U) && \
     (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 1U))
#error "EMERGENCY_SINGLE_PULSE_DEBUG_MODE must be 0 or 1"
#endif

#if ((USART2_DEBUG_ENABLED != 0U) && (USART2_DEBUG_ENABLED != 1U))
#error "USART2_DEBUG_ENABLED must be 0 or 1"
#endif

#endif /* BUILD_OPTIONS_H */
