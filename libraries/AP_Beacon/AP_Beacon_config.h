#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_BEACON_ENABLED
#define AP_BEACON_ENABLED 0 // 1 AKGL
#endif

#ifndef AP_BEACON_BACKEND_DEFAULT_ENABLED
#define AP_BEACON_BACKEND_DEFAULT_ENABLED AP_BEACON_ENABLED
#endif

#ifndef AP_BEACON_MARVELMIND_ENABLED
#define AP_BEACON_MARVELMIND_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_NOOPLOOP_ENABLED
#define AP_BEACON_NOOPLOOP_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_POZYX_ENABLED
#define AP_BEACON_POZYX_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_SITL_ENABLED
#define AP_BEACON_SITL_ENABLED (AP_BEACON_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
