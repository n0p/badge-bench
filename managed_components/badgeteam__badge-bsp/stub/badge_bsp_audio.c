// Board support package API: Generic stub implementation
// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "bsp/audio.h"
#include "esp_err.h"

#include <stdint.h>

esp_err_t __attribute__((weak)) bsp_audio_initialize(void) {
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t __attribute__((weak)) bsp_audio_get_volume(float *out_percentage) {
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t __attribute__((weak)) bsp_audio_set_volume(float percentage) {
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t __attribute__((weak)) bsp_audio_set_amplifier(bool enable) {
    return ESP_ERR_NOT_SUPPORTED;
}
