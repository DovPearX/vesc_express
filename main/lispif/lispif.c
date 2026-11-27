/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2023 Joel Svensson    svenssonjoel@yahoo.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <malloc.h>
#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "eval_cps.h"
#include "main.h"
#include "lispif.h"
#include "lispif_vesc_extensions.h"
#include "modules/can_extensions.h"
#include "commands.h"
#include "buffer.h"
#include "lispbm.h"
#include "lbm_image.h"
#include "mempools.h"
#include "packet.h"
#include "flash_helper.h"
#include "lbm_prof.h"
#include "esp_timer.h"
#include "esp_system.h"

#define GC_STACK_SIZE			160
#define PRINT_STACK_SIZE		128
#ifndef EXTENSION_STORAGE_SIZE
#define EXTENSION_STORAGE_SIZE	192
#endif
#define PROF_DATA_NUM 30

static size_t heap_size = 0;
static size_t mem_size = 0;
static size_t bitmap_size = 0;

static lbm_cons_t *heap;
static uint32_t *memory_array;
static uint32_t *bitmap_array;
// LispBM stores constant symbols in an image, even without flash snapshots.
static _Alignas(8) uint32_t ram_image[2048];
static lbm_extension_t extension_storage[EXTENSION_STORAGE_SIZE];

static lbm_buffered_channel_state_t stream_state;
static lbm_char_channel_t stream_channel;
static bool stream_active;
static int32_t stream_size;
static int32_t stream_next_offset;
static int32_t stream_last_offset = -1;

static TaskHandle_t eval_task = 0;
static atomic_bool lisp_thd_running;
static SemaphoreHandle_t lbm_mutex;

static int repl_cid = -1;
static lbm_cid repl_cid_for_buffer = -1;
static char *repl_buffer = 0;

static esp_timer_handle_t prof_timer;
static void prof_timer_callback(void* arg);
static lbm_prof_t prof_data[PROF_DATA_NUM];
static volatile bool prof_running = false;
static const esp_timer_create_args_t periodic_timer_args = {
		.callback = &prof_timer_callback,
};

static void sleep_callback(uint32_t us);
static void eval_thread(void *arg);
static bool lispif_restart(bool print, bool load_code);

#define LBM_MEMORY_SIZE_KB(kb) LBM_MEMORY_SIZE_64BYTES_TIMES_X((kb * 16))
#define LBM_BITMAP_SIZE_KB(kb) LBM_MEMORY_BITMAP_SIZE((kb * 16))

#if CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32S31
#define LBM_PSRAM_HEAP_BYTES   (4096 * 64)
#define LBM_PSRAM_MEMORY_KB    2048
#define LBM_PSRAM_BITMAP_KB    2048
#else
#define LBM_PSRAM_HEAP_BYTES   (4096 * 16)
#define LBM_PSRAM_MEMORY_KB    512
#define LBM_PSRAM_BITMAP_KB    512
#endif


void lispif_init(void) {
#ifndef CONFIG_SPIRAM
#ifdef CONFIG_IDF_TARGET_ESP32S3
	heap_size = (4096 + 512);
	mem_size = LBM_MEMORY_SIZE_KB(48);
	bitmap_size = LBM_BITMAP_SIZE_KB(48);
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
	heap_size = (2048 + 512);
	mem_size = LBM_MEMORY_SIZE_KB(32);
	bitmap_size = LBM_BITMAP_SIZE_KB(32);
#elif CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32S31
	heap_size = (4096 + 512);
	mem_size = LBM_MEMORY_SIZE_KB(32);
	bitmap_size = LBM_BITMAP_SIZE_KB(32);
#else
	#error "Unsupported target"
#endif

#ifdef HW_LBM_HEAP_SIZE
	heap_size = HW_LBM_HEAP_SIZE;
#endif
#ifdef HW_LBM_MEMORY_KB
	mem_size = LBM_MEMORY_SIZE_KB(HW_LBM_MEMORY_KB);
	bitmap_size = LBM_BITMAP_SIZE_KB(HW_LBM_MEMORY_KB);
#endif

#endif

#ifdef CONFIG_SPIRAM
	heap_size = LBM_PSRAM_HEAP_BYTES / sizeof(lbm_cons_t);
	heap = heap_caps_aligned_alloc(8, heap_size * sizeof(lbm_cons_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	mem_size = LBM_MEMORY_SIZE_KB(LBM_PSRAM_MEMORY_KB);
	bitmap_size = LBM_BITMAP_SIZE_KB(LBM_PSRAM_BITMAP_KB);
	memory_array = heap_caps_malloc(mem_size * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	bitmap_array = heap_caps_malloc(bitmap_size * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
	memory_array = heap_caps_malloc(mem_size * sizeof(uint32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
	bitmap_array = heap_caps_malloc(bitmap_size * sizeof(uint32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
	heap = memalign(8, heap_size * sizeof(lbm_cons_t));
#endif

	lbm_mutex = xSemaphoreCreateMutex();
	if (!heap || !memory_array || !bitmap_array || !lbm_mutex) {
		ESP_LOGE("lispif", "Could not allocate LispBM memory");
		free(heap);
		free(memory_array);
		free(bitmap_array);
		heap = NULL;
		memory_array = NULL;
		bitmap_array = NULL;
		return;
	}

	if (flash_helper_code_size(CODE_IND_LISP) > 0) {
		lispif_restart(false, true);
	}
}

static void print_ctx_info(eval_context_t *ctx, void *arg1, void *arg2) {
	(void) arg1;
	(void) arg2;

	char output[128];

	int print_ret = lbm_print_value(output, sizeof(output), ctx->r);

	commands_printf_lisp("--------------------------------");
	commands_printf_lisp("ContextID: %u", ctx->id);

	if (ctx->name) {
		commands_printf_lisp("Context Name: %s", ctx->name);
	}

	commands_printf_lisp("Stack SP: %u",  ctx->K.sp);
	commands_printf_lisp("Stack SP max: %u", lbm_get_max_stack(&ctx->K));
	commands_printf_lisp("Result%s: %s", print_ret ? "" : " (trunc)", output);
}

static void sym_it(const char *str) {
	bool sym_name_flash = lbm_symbol_in_flash((char *)str);
	bool sym_entry_flash = lbm_symbol_list_entry_in_flash((char *)str);
	commands_printf_lisp("[Name: %s, Entry: %s]: %s\n",
			sym_name_flash ? "FLASH" : "L_MEM",
					sym_entry_flash ? "FLASH" : "L_MEM",
							str);
}

static void prof_timer_callback(void* arg) {
	(void)arg;
	lbm_prof_sample();
}

static bool pause_eval(uint32_t num_free, uint32_t timeout_ms) {
	if (!lisp_thd_running) {
		return false;
	}

	TickType_t start = xTaskGetTickCount();

	if (num_free > 0) {
		lbm_pause_eval_with_gc(num_free);
	} else {
		lbm_pause_eval();
	}

	while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED) {
		if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(timeout_ms)) {
			return false;
		}
		vTaskDelay(1);
	}

	return true;
}

void lispif_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	COMM_PACKET_ID packet_id;

	if (len == 0) {
		return;
	}

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_LISP_SET_RUNNING: {
		bool ok = false;
		if (len != 0) {
			if (data[0]) {
				ok = lispif_restart(true, true);
			} else {
				ok = !lisp_thd_running || pause_eval(0, 2000);
			}
		}

		int32_t ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = ok;
		reply_func(send_buffer, ind);
	} break;


	case COMM_LISP_GET_STATS: {
		float cpu_use = 0.0;
		float heap_use = 0.0;
		float mem_use = 0.0;

		if (lisp_thd_running) {
			#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_USE_TRACE_FACILITY
			uint32_t timeTot = 0;
#ifdef portALT_GET_RUN_TIME_COUNTER_VALUE
			portALT_GET_RUN_TIME_COUNTER_VALUE( timeTot );
#else
			timeTot = portGET_RUN_TIME_COUNTER_VALUE();
#endif
			if (timeTot > 0) {
				TaskStatus_t stat;
				vTaskGetInfo(eval_task, &stat, pdFALSE, 0);
				static uint32_t time_last = 0;
				static uint32_t time_task_last = 0;
				if (timeTot != time_last && stat.ulRunTimeCounter >= time_task_last) {
					cpu_use = 100.0f * (float)(stat.ulRunTimeCounter - time_task_last) /
							(float)(timeTot - time_last);
				}
				time_last = timeTot;
				time_task_last = stat.ulRunTimeCounter;
			}
			#endif
		}

		bool print_all = true;
		if (len > 0) {
			print_all = data[0];
		}

		bool was_paused = lbm_get_eval_state_requested() == EVAL_CPS_STATE_PAUSED;
		bool paused = pause_eval(0, 2000);
		if (paused) {
			heap_use = 100.0f * (heap_size - lbm_heap_num_free()) / heap_size;
			mem_use = 100.0f * (lbm_memory_num_words() - lbm_memory_num_free()) / lbm_memory_num_words();
		}

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		int32_t ind = 0;

		send_buffer_global[ind++] = packet_id;
		buffer_append_float16(send_buffer_global, cpu_use, 1e2, &ind);
		buffer_append_float16(send_buffer_global, heap_use, 1e2, &ind);
		buffer_append_float16(send_buffer_global, mem_use, 1e2, &ind);

		// Stack. Currently unused
		buffer_append_float16(send_buffer_global, 0, 1e2, &ind);

		// Result. Currently unused.
		send_buffer_global[ind++] = '\0';

		if (paused) {
			lbm_value *glob_env = lbm_get_global_env();
			for (int i = 0; i < GLOBAL_ENV_ROOTS; i ++) {
				if (ind > 300) {
					break;
				}

				lbm_value curr = glob_env[i];
				while (lbm_type_of(curr) == LBM_TYPE_CONS) {
					lbm_value key_val = lbm_car(curr);
					if (lbm_type_of(lbm_car(key_val)) == LBM_TYPE_SYMBOL && lbm_is_number(lbm_cdr(key_val))) {
						const char *name = lbm_get_name_by_symbol(lbm_dec_sym(lbm_car(key_val)));

						if (name && (print_all ||
								((name[0] == 'v' || name[0] == 'V') &&
										(name[1] == 't' || name[1] == 'T')))) {
							size_t name_len = strlen(name) + 1;
							if (name_len + sizeof(uint32_t) > PACKET_MAX_PL_LEN - ind) {
								break;
							}
							memcpy(send_buffer_global + ind, name, name_len);
							ind += name_len;
							buffer_append_float32_auto(send_buffer_global, lbm_dec_as_float(lbm_cdr(key_val)), &ind);
						}
					}

					if (ind > 300) {
						break;
					}

					curr = lbm_cdr(curr);
				}
			}
		}

		if (lisp_thd_running && !was_paused) {
			lbm_continue_eval();
		}

		reply_func(send_buffer_global, ind);
		mempools_free_packet_buffer(send_buffer_global);
	} break;

	case COMM_LISP_REPL_CMD: {
		if (!lisp_thd_running) {
			lispif_restart(true, false);
		}

		if (lisp_thd_running) {
			// The command worker reserves an extra byte after each packet.
			data[len] = '\0';
			char *str = (char*)data;

			if (!data[0]) {
				commands_printf_lisp(">");
			} else if (len >= 5 && strncmp(str, ":help", 5) == 0) {
				commands_printf_lisp("== Special Commands ==");
				commands_printf_lisp(
						":help\n"
						"  Print this help text");
				commands_printf_lisp(
						":info\n"
						"  Print info about memory usage, allocated arrays and garbage collection");
				commands_printf_lisp(
						":prof start\n"
						"  Start profiler");
				commands_printf_lisp(
						":prof stop\n"
						"  Stop profiler");
				commands_printf_lisp(
						":prof report\n"
						"  Print profiler report");
				commands_printf_lisp(
						":env\n"
						"  Print current environment and variables");
				commands_printf_lisp(
						":ctxs\n"
						"  Print context (threads) info");
				commands_printf_lisp(
						":symbols\n"
						"  Print symbol names");
				commands_printf_lisp(
						":reset\n"
						"  Reset LBM");
				commands_printf_lisp(
						":pause\n"
						"  Pause LBM");
				commands_printf_lisp(
						":continue\n"
						"  Continue running LBM");
				commands_printf_lisp(
						":undef <symbol_name>\n"
						"  Undefine symbol");
				commands_printf_lisp(
						":verb\n"
						"  Toggle verbose error messages");
				commands_printf_lisp(
						":state\n"
						"  Print evaluator state");
				commands_printf_lisp(" ");
				commands_printf_lisp("Anything else will be evaluated as an expression in LBM.");
				commands_printf_lisp(" ");
			} else if (len >= 5 && strncmp(str, ":info", 5) == 0) {
				commands_printf_lisp("--(LISP HEAP)--\n");
				commands_printf_lisp("Heap size: %u Bytes\n", heap_size * 8);
				commands_printf_lisp("Used cons cells: %d\n", heap_size - lbm_heap_num_free());
				commands_printf_lisp("Free cons cells: %d\n", lbm_heap_num_free());
				commands_printf_lisp("GC counter: %d\n", lbm_heap_state.gc_num);
				commands_printf_lisp("Recovered: %d\n", lbm_heap_state.gc_recovered);
				commands_printf_lisp("Recovered arrays: %u\n", lbm_heap_state.gc_recovered_arrays);
				commands_printf_lisp("Marked: %d\n", lbm_heap_state.gc_marked);
				commands_printf_lisp("GC SP max: %u (size %u)\n", lbm_get_max_stack(&lbm_heap_state.gc_stack), lbm_heap_state.gc_stack.size);
				commands_printf_lisp("Global env cells: %u\n", lbm_get_global_env_size());
				commands_printf_lisp("--(Symbol and Array memory)--\n");
				commands_printf_lisp("Memory size: %u bytes\n", lbm_memory_num_words() * 4);
				commands_printf_lisp("Memory free: %u bytes\n", lbm_memory_num_free() * 4);
				commands_printf_lisp("Longest block free: %u bytes\n", lbm_memory_longest_free() * 4);
				commands_printf_lisp("Allocated arrays: %u\n", lbm_heap_state.num_alloc_arrays);
				commands_printf_lisp("Symbol table size: %u Bytes\n", lbm_get_symbol_table_size());
				commands_printf_lisp("Symbol table size flash: %u Bytes\n", lbm_get_symbol_table_size_flash());
				commands_printf_lisp("Symbol name size: %u Bytes\n", lbm_get_symbol_table_size_names());
				commands_printf_lisp("Symbol name size flash: %u Bytes\n", lbm_get_symbol_table_size_names_flash());
				commands_printf_lisp("Extensions: %u, max %u\n", lbm_get_num_extensions(), lbm_get_max_extensions());
			} else if (strncmp(str, ":prof start", 11) == 0) {
				if (prof_running) {
					lbm_prof_init(prof_data, PROF_DATA_NUM);
					commands_printf_lisp("Profiler restarted\n");
				} else {
					lbm_prof_init(prof_data, PROF_DATA_NUM);
					if (prof_timer || esp_timer_create(&periodic_timer_args, &prof_timer) == ESP_OK) {
						prof_running = esp_timer_start_periodic(prof_timer, 571) == ESP_OK;
					}
					commands_printf_lisp("%s\n", prof_running ? "Profiler started" : "Could not start profiler");
				}
			} else if (strncmp(str, ":prof stop", 10) == 0) {
				if (prof_running) {
					prof_running = false;
					esp_timer_stop(prof_timer);
				}
				commands_printf_lisp("Profiler stopped. Issue command ':prof report' for statistics\n");
			} else if (strncmp(str, ":prof report", 12) == 0) {
				lbm_uint num_sleep = lbm_prof_get_num_sleep_samples();
				lbm_uint num_system = lbm_prof_get_num_system_samples();
				lbm_uint tot_samples = lbm_prof_get_num_samples();
				lbm_uint tot_gc = 0;
				commands_printf_lisp("CID\tName\tSamples\t%%Load\t%%GC");
				for (int i = 0; i < PROF_DATA_NUM; i ++) {
					if (prof_data[i].cid == -1) break;
					tot_gc += prof_data[i].gc_count;
					commands_printf_lisp("%d\t%s\t%u\t%.3f\t%.3f",
							prof_data[i].cid,
							prof_data[i].name,
							prof_data[i].count,
							(double)(100.0 * ((float)prof_data[i].count) / (float) tot_samples),
							(double)(100.0 * ((float)prof_data[i].gc_count) / (float)prof_data[i].count));
				}
				commands_printf_lisp(" ");
				commands_printf_lisp("GC:\t%u\t%f%%\n", tot_gc, (double)(100.0 * ((float)tot_gc / (float)tot_samples)));
				commands_printf_lisp("System:\t%u\t%f%%\n", num_system, (double)(100.0 * ((float)num_system / (float)tot_samples)));
				commands_printf_lisp("Sleep:\t%u\t%f%%\n", num_sleep, (double)(100.0 * ((float)num_sleep / (float)tot_samples)));
				commands_printf_lisp("Total:\t%u samples\n", tot_samples);
			} else if (strncmp(str, ":env", 4) == 0) {
				if (pause_eval(0, 1000)) {
					lbm_value *glob_env = lbm_get_global_env();
					char output[128];
					for (int i = 0; i < GLOBAL_ENV_ROOTS; i ++) {
						lbm_value curr = glob_env[i];
						while (lbm_type_of(curr) == LBM_TYPE_CONS) {
							lbm_print_value(output, sizeof(output), lbm_car(curr));
							curr = lbm_cdr(curr);

							commands_printf_lisp("  %s", output);
						}
					}
				}
			} else if (strncmp(str, ":ctxs", 5) == 0) {
				commands_printf_lisp("****** Running contexts ******");
				lbm_running_iterator(print_ctx_info, NULL, NULL);
				commands_printf_lisp("****** Blocked contexts ******");
				lbm_blocked_iterator(print_ctx_info, NULL, NULL);
			} else if (strncmp(str, ":symbols", 8) == 0) {
				if (pause_eval(0, 1000)) {
					lbm_symrepr_name_iterator(sym_it);
					commands_printf_lisp(" ");
				}
			} else if (strncmp(str, ":reset", 6) == 0) {
				commands_printf_lisp(lispif_restart(true, flash_helper_code_size(CODE_IND_LISP) > 0) ?
						"Reset OK\n\n" : "Reset Failed\n\n");
			} else if (strncmp(str, ":pause", 6) == 0) {
				if (pause_eval(30, 1000)) {
					commands_printf_lisp("Evaluator paused\n");
				}
			} else if (strncmp(str, ":continue", 9) == 0) {
				lbm_continue_eval();
			} else if (strncmp(str, ":undef", 6) == 0) {
				if (pause_eval(30, 1000)) {
					char *sym = len > 7 ? str + 7 : "";
					commands_printf_lisp("undefining: %s", sym);
					commands_printf_lisp("%s", lbm_undefine(sym) ? "Cleared bindings" : "No definition found");
					lbm_continue_eval();
				}
			} else if (strncmp(str, ":verb", 5) == 0) {
				static bool verbose_now = false;
				verbose_now = !verbose_now;
				lbm_set_verbose(verbose_now);
				commands_printf_lisp("Verbose errors %s", verbose_now ? "Enabled" : "Disabled");
			} else if (strncmp(str, ":state", 6) == 0) {
				lbm_uint state = lbm_get_eval_state();
				switch (state) {
					case EVAL_CPS_STATE_DEAD:
						commands_printf_lisp("DEAD\n");
						break;
					case EVAL_CPS_STATE_PAUSED:
						commands_printf_lisp("PAUSED\n");
						break;
					case EVAL_CPS_STATE_NONE:
						commands_printf_lisp("NO STATE\n");
						break;
					case EVAL_CPS_STATE_RUNNING:
						commands_printf_lisp("RUNNING\n");
						break;
					case EVAL_CPS_STATE_KILL:
						commands_printf_lisp("KILLING\n");
						break;
				}
			} else if (pause_eval(30, 1000)) {
				if (repl_buffer) {
					commands_printf_lisp("LispBM is busy");
				} else {
					repl_buffer = lbm_malloc_reserve(len + 1);
					if (repl_buffer) {
						static lbm_string_channel_state_t state;
						static lbm_char_channel_t channel;
						memcpy(repl_buffer, data, len + 1);
						lbm_create_string_char_channel(&state, &channel, repl_buffer);
						repl_cid = lbm_load_and_eval_expression(&channel);
						repl_cid_for_buffer = repl_cid;
						if (repl_cid < 0) {
							lbm_free(repl_buffer);
							repl_buffer = NULL;
							commands_printf_lisp("Could not queue expression");
						}
					} else {
						commands_printf_lisp("Not enough memory");
					}
				}
				lbm_continue_eval();
			} else {
				lbm_continue_eval();
				commands_printf_lisp("Could not pause");
			}
		} else {
			commands_printf_lisp("LispBM is not running");
		}
	} break;

	case COMM_LISP_STREAM_CODE: {
		if (len < 9) {
			break;
		}
		int32_t index = 0;
		int32_t offset = buffer_get_int32(data, &index);
		int32_t total = buffer_get_int32(data, &index);
		uint8_t restart = data[index++];
		int16_t result = -15;

		if (offset >= 0 && offset <= total && len - index <= (uint32_t)(total - offset)) {
			if (offset == stream_last_offset && total == stream_size) {
				result = 0;
			} else if (offset == 0) {
				if ((restart || !lisp_thd_running) && !lispif_restart(true, restart == 2)) {
					result = -1;
				} else {
					if (stream_active) {
						lbm_channel_writer_close(&stream_channel);
						lbm_continue_eval();
						TickType_t start = xTaskGetTickCount();
						while (!lbm_channel_reader_is_closed(&stream_channel) &&
								xTaskGetTickCount() - start < pdMS_TO_TICKS(1500)) {
							vTaskDelay(1);
						}
					}
					if (stream_active && !lbm_channel_reader_is_closed(&stream_channel)) {
						result = -2;
					} else if (!pause_eval(30, 1000)) {
						result = -3;
						lbm_continue_eval();
					} else {
						lbm_create_buffered_char_channel(&stream_state, &stream_channel);
						stream_active = lbm_load_and_eval_program(&stream_channel, "main-s") > 0;
						stream_size = total;
						stream_next_offset = 0;
						stream_last_offset = -1;
						result = stream_active ? 0 : -4;
						lbm_continue_eval();
					}
				}
			} else if (stream_active && offset == stream_next_offset && total == stream_size) {
				result = 0;
			}

			if (result == 0 && offset != stream_last_offset) {
				TickType_t last_progress = xTaskGetTickCount();
				while (index < len) {
					int status = lbm_channel_write(&stream_channel, data[index]);
					if (status == CHANNEL_SUCCESS) {
						index++;
						last_progress = xTaskGetTickCount();
					} else if (status == CHANNEL_READER_CLOSED) {
						result = -6;
						break;
					} else if (xTaskGetTickCount() - last_progress >= pdMS_TO_TICKS(1500)) {
						result = -5;
						break;
					} else {
						vTaskDelay(1);
					}
				}
				if (result == 0) {
					stream_last_offset = offset;
					stream_next_offset = offset + len - 9;
				}
				if (result != 0 || stream_next_offset == stream_size) {
					lbm_channel_writer_close(&stream_channel);
				}
				if (result != 0) {
					stream_next_offset = -1;
					stream_last_offset = -1;
				}
			}
		}

		uint8_t reply[7] = {packet_id};
		index = 1;
		buffer_append_int32(reply, offset, &index);
		buffer_append_int16(reply, result, &index);
		reply_func(reply, sizeof(reply));
	} break;

	case COMM_CUSTOM_APP_DATA:
		if (lisp_thd_running) {
			lispif_process_custom_app_data(data, len);
		}
		break;

	default:
		break;
	}
}

static void done_callback(eval_context_t *ctx) {
	lispif_events_context_done(ctx->id);
	if (ctx->id == repl_cid) {
		char output[128];
		lbm_print_value(output, sizeof(output) - 1, ctx->r);
		output[sizeof(output) - 1] = '\0';
		commands_printf_lisp("> %s", output);
		repl_cid = -1;
	}

	if (ctx->id == repl_cid_for_buffer) {
		lbm_free(repl_buffer);
		repl_buffer = NULL;
		repl_cid_for_buffer = -1;
	}
}

bool lispif_stop(void) {
	if (lisp_thd_running) {
		xSemaphoreTake(lbm_mutex, portMAX_DELAY);
		lbm_kill_eval();
		TickType_t start = xTaskGetTickCount();
		while (lisp_thd_running && xTaskGetTickCount() - start < pdMS_TO_TICKS(2000)) {
			vTaskDelay(1);
		}
		bool stopped = !lisp_thd_running;
		xSemaphoreGive(lbm_mutex);
		if (!stopped) {
			return false;
		}
	}

	if (prof_running) {
		esp_timer_stop(prof_timer);
		prof_running = false;
	}
	if (repl_buffer) {
		lbm_free(repl_buffer);
	}
	repl_buffer = NULL;
	repl_cid = -1;
	repl_cid_for_buffer = -1;
	stream_active = false;
	stream_last_offset = -1;
	return true;
}

static bool write_ram_image(uint32_t word, int32_t index, bool const_heap) {
	(void)const_heap;
	if (index < 0 || (size_t)index >= sizeof(ram_image) / sizeof(ram_image[0])) {
		return false;
	}
	ram_image[index] = word;
	return true;
}

static bool lispif_restart(bool print, bool load_code) {
	if (!heap || !lispif_stop()) {
		return false;
	}

	if (!lbm_init(heap, heap_size, memory_array, mem_size, bitmap_array,
			bitmap_size, GC_STACK_SIZE, PRINT_STACK_SIZE, extension_storage,
			sizeof(extension_storage) / sizeof(extension_storage[0])) ||
			!lbm_eval_init_events(30)) {
		ESP_LOGE("lispif", "Could not initialize LispBM");
		return false;
	}

	lbm_set_usleep_callback(sleep_callback);
	lbm_set_printf_callback(commands_printf_lisp);
	lbm_set_ctx_done_callback(done_callback);
	lbm_set_eval_step_quota(50);
	memset(ram_image, 0xff, sizeof(ram_image));
	lbm_image_init(ram_image, sizeof(ram_image) / sizeof(ram_image[0]), write_ram_image);
	lbm_image_create(NULL);
	if (!lbm_image_boot()) {
		ESP_LOGE("lispif", "Could not initialize LispBM image");
		return false;
	}
	lbm_add_eval_symbols();
	lispif_load_vesc_extensions();

	const uint8_t *code = load_code ? flash_helper_code_data_ptr(CODE_IND_LISP) : NULL;
	if (code) {
		static lbm_string_channel_state_t state;
		static lbm_char_channel_t channel;
		size_t len = strnlen((const char *)code, flash_helper_code_size(CODE_IND_LISP));
		if (len != 0) {
			if (print) {
				commands_printf_lisp("Parsing %u characters", (unsigned int)len);
			}
			lbm_create_string_char_channel_size(&state, &channel, (char *)code, len);
			if (lbm_load_and_eval_program_incremental(&channel, "main-u") <= 0) {
				return false;
			}
		}
	}

	if (xTaskCreate(eval_thread, "lbm_eval", 4096, NULL, 6, &eval_task) != pdPASS) {
		ESP_LOGE("lispif", "Could not create LispBM task");
		return false;
	}
	lisp_thd_running = true;
	return true;
}

void lispif_process_can(unsigned bus, uint32_t id, uint8_t *data, uint8_t len, bool extended) {
	if (lisp_thd_running && lispif_can_event_enabled(bus, extended) &&
			xSemaphoreTake(lbm_mutex, 0) == pdTRUE) {
		if (lisp_thd_running) {
			lispif_can_event(bus, id, data, len, extended);
		}
		xSemaphoreGive(lbm_mutex);
	}
}

static void sleep_callback(uint32_t us) {
	TickType_t ticks = us / (1000000 / configTICK_RATE_HZ);
	vTaskDelay(ticks ? ticks : 1);
}

static void eval_thread(void *arg) {
	(void)arg;
	lbm_run_eval();
	lisp_thd_running = false;
	vTaskDelete(NULL);
}
